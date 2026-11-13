"""
Detección de piezas en L con ESP32-CAM.
Entrega la posición (x, y) en mm y la rotación phi en grados, en el MARCO DEL
ROBOT (el mismo de la cinemática inversa del ESP32: x adelante, y a la
izquierda, phi antihorario visto desde arriba).

Requisitos:  pip install opencv-python numpy pyserial

Uso (en este orden la primera vez):
  1) python detectar_piezas.py color          -> ajustar el color de la pieza
  2) python detectar_piezas.py plano          -> calibrar píxeles -> mm
  3) python detectar_piezas.py                -> detección en vivo
     python detectar_piezas.py --serial COM8  -> y con la tecla 'e' envía
                                                 "P x,y,phi" al robot

Convención de phi: ángulo del vector que va de la esquina EXTERIOR de la L a
la esquina INTERIOR (la del ángulo cóncavo), medido desde el eje x del robot.
Es único para cualquier orientación de 0 a 360°.
"""
import argparse
import json
import math
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

# ========================= CONFIGURACIÓN =========================
IP_CAMARA   = "10.213.65.212"                       # la IP que imprime el ESP32-CAM al arrancar
URL_STREAM  = f"http://{IP_CAMARA}:81/stream"
URL_CAPTURA = f"http://{IP_CAMARA}/capture"          # respaldo si el stream falla
ARCHIVO_CAL = Path("calibracion_camara.json")
AREA_MIN    = 800                            # [px] contornos más pequeños se ignoran
HSV_BAJO    = [0, 0, 180]                   # color inicial (amarillo); se ajusta con "color"
HSV_ALTO    = [179, 50, 255]
# =================================================================


# ------------------------------ Cámara ------------------------------
class Camara:
    def __init__(self, ip_camara=IP_CAMARA):
        base_url = f"http://{ip_camara.strip().rstrip('/') }"
        self.url_stream = f"{base_url}:81/stream"
        self.url_captura = f"{base_url}/capture"
        self.cap = cv2.VideoCapture(self.url_stream)
        self.usar_stream = self.cap.isOpened()
        if not self.usar_stream:
            print("Stream no disponible, se usará /capture (más lento).")

    def leer(self):
        if self.usar_stream:
            ok, frame = self.cap.read()
            if ok:
                return frame
        try:
            with urllib.request.urlopen(self.url_captura, timeout=3) as r:
                datos = np.frombuffer(r.read(), np.uint8)
            return cv2.imdecode(datos, cv2.IMREAD_COLOR)
        except Exception as e:
            print("Error leyendo la cámara:", e)
            time.sleep(0.5)
            return None


# --------------------------- Calibración ----------------------------
def cargar_cal():
    if ARCHIVO_CAL.exists():
        return json.loads(ARCHIVO_CAL.read_text())
    return {}


def guardar_cal(cal):
    ARCHIVO_CAL.write_text(json.dumps(cal, indent=2))
    print(f"Calibración guardada en {ARCHIVO_CAL}")


def a_mundo(H, pts):
    """Píxeles -> mm en el marco del robot (o deja píxeles si no hay H)."""
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    if H is None:
        return pts.reshape(-1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


# --------------------------- Visión ---------------------------------
def mascara(frame, bajo, alto):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(bajo), np.array(alto))
    k = np.ones((5, 5), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    return m


def esquinas_L(contorno):
    """Devuelve (esquina_exterior, punto hacia la esquina interior) en píxeles, o None."""
    peri = cv2.arcLength(contorno, True)
    poly = None
    for eps in (0.02, 0.03, 0.015, 0.04, 0.01):
        p = cv2.approxPolyDP(contorno, eps * peri, True).reshape(-1, 2).astype(float)
        if len(p) == 6:
            poly = p
            break
    if poly is None:
        return None

    # Sentido de recorrido del polígono (fórmula del área con signo)
    x, y = poly[:, 0], poly[:, 1]
    area = np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)

    # Giro en cada vértice: la L tiene 5 vértices convexos y 1 cóncavo
    concavos = []
    for i in range(6):
        a, b, c = poly[i - 1], poly[i], poly[(i + 1) % 6]
        u, v = b - a, c - b
        giro = u[0] * v[1] - u[1] * v[0]
        if np.sign(giro) != np.sign(area):
            concavos.append(i)
    if len(concavos) != 1:
        return None

    ic = concavos[0]
    io = (ic + 3) % 6          # la esquina exterior es la opuesta a la cóncava
    ext, inte = poly[io], poly[ic]

    # Dirección más precisa: bisectriz de los dos bordes exteriores (son los
    # lados largos, así que su ángulo se estima mejor que el de las esquinas)
    u1 = poly[(io + 1) % 6] - ext
    u2 = poly[io - 1] - ext
    b = u1 / np.linalg.norm(u1) + u2 / np.linalg.norm(u2)
    b = b / np.linalg.norm(b)
    punta = ext + b * np.linalg.norm(inte - ext)   # punto sobre la bisectriz
    return ext, punta


def pose_pieza(contorno, esquinas, H):
    """Centroide (x, y) y phi en el marco del robot."""
    cont_mm = a_mundo(H, contorno.reshape(-1, 2)).reshape(-1, 1, 2).astype(np.float32)
    M = cv2.moments(cont_mm)
    if abs(M["m00"]) < 1e-9:
        return None
    cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
    ext, inte = a_mundo(H, np.array(esquinas))
    phi = math.degrees(math.atan2(inte[1] - ext[1], inte[0] - ext[0]))
    return cx, cy, phi


# --------------------------- Modo: color -----------------------------
def modo_color(ip_camara):
    cal = cargar_cal()
    bajo = cal.get("hsv_bajo", HSV_BAJO)
    alto = cal.get("hsv_alto", HSV_ALTO)
    cam = Camara(ip_camara)
    cv2.namedWindow("Color")
    nombres = ["H min", "S min", "V min", "H max", "S max", "V max"]
    maximos = [179, 255, 255, 179, 255, 255]
    for n, val, mx in zip(nombres, bajo + alto, maximos):
        cv2.createTrackbar(n, "Color", int(val), mx, lambda _: None)
    print("Ajusta hasta que solo la pieza quede en blanco. 's' guarda, 'q' sale.")
    while True:
        frame = cam.leer()
        if frame is None:
            continue
        vals = [cv2.getTrackbarPos(n, "Color") for n in nombres]
        m = mascara(frame, vals[:3], vals[3:])
        vista = np.hstack([frame, cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)])
        cv2.imshow("Color", vista)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("s"):
            cal["hsv_bajo"], cal["hsv_alto"] = vals[:3], vals[3:]
            guardar_cal(cal)
        elif k == ord("q"):
            break
    cv2.destroyAllWindows()


# --------------------------- Modo: plano -----------------------------
def modo_plano(ip_camara):
    """Clic en 4 o más puntos de la imagen y escribe sus coordenadas en mm."""
    cal = cargar_cal()
    cam = Camara(ip_camara)
    frame = None
    while frame is None:
        frame = cam.leer()
    clics = []

    def al_clic(evento, x, y, *_):
        if evento == cv2.EVENT_LBUTTONDOWN:
            clics.append((x, y))

    cv2.namedWindow("Plano")
    cv2.setMouseCallback("Plano", al_clic)
    pts_img, pts_mm = [], []
    print("Haz clic en un punto de referencia y escribe sus coordenadas en la consola.")
    print("Mínimo 4 puntos, que no estén en línea. 'c' calcula y guarda, 'q' sale.")
    while True:
        vista = frame.copy()
        for (u, v), (xm, ym) in zip(pts_img, pts_mm):
            cv2.circle(vista, (u, v), 5, (0, 0, 255), -1)
            cv2.putText(vista, f"({xm:.0f},{ym:.0f})", (u + 6, v - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow("Plano", vista)
        k = cv2.waitKey(30) & 0xFF
        if clics:
            u, v = clics.pop(0)
            txt = input(f"Píxel ({u},{v}) -> coordenadas del robot 'x,y' en mm: ")
            try:
                xm, ym = (float(s) for s in txt.split(","))
                pts_img.append((u, v))
                pts_mm.append((xm, ym))
            except ValueError:
                print("Formato inválido, usa por ejemplo: 150,80")
        if k == ord("c") and len(pts_img) >= 4:
            H, _ = cv2.findHomography(np.float32(pts_img), np.float32(pts_mm))
            cal["H"] = H.tolist()
            guardar_cal(cal)
            break
        if k == ord("q"):
            break
    cv2.destroyAllWindows()


# --------------------------- Modo: detección --------------------------
def modo_detectar(puerto_serial, ip_camara):
    cal = cargar_cal()
    bajo = cal.get("hsv_bajo", HSV_BAJO)
    alto = cal.get("hsv_alto", HSV_ALTO)
    H = np.array(cal["H"]) if "H" in cal else None
    unidad = "mm" if H is not None else "px"
    if H is None:
        print("AVISO: no hay calibración de plano; se reporta en píxeles.")

    ser = None
    if puerto_serial:
        import serial
        ser = serial.Serial(puerto_serial, 115200, timeout=0.1)
        time.sleep(2)                          # la ESP32 se reinicia al abrir el puerto

    cam = Camara(ip_camara)
    ultima_impresion = 0
    print("'e' envía la pose de la pieza más grande, 'q' sale.")
    while True:
        frame = cam.leer()
        if frame is None:
            continue
        m = mascara(frame, bajo, alto)
        contornos, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        piezas = []
        for c in contornos:
            area = cv2.contourArea(c)
            if area < AREA_MIN:
                continue
            esq = esquinas_L(c)
            if esq is None:
                cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)   # rojo: no parece una L
                continue
            pose = pose_pieza(c, esq, H)
            if pose is None:
                continue
            piezas.append((area, pose))
            x, y, phi = pose
            ext, inte = (tuple(int(v) for v in p) for p in esq)
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
            cv2.arrowedLine(frame, ext, inte, (255, 0, 0), 2, tipLength=0.25)
            cv2.putText(frame, f"x={x:.0f} y={y:.0f} phi={phi:.0f}", (ext[0], ext[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        piezas.sort(key=lambda p: p[0], reverse=True)
        if piezas and time.time() - ultima_impresion > 0.5:
            x, y, phi = piezas[0][1]
            print(f"Pieza: x={x:7.1f} {unidad}  y={y:7.1f} {unidad}  phi={phi:6.1f}°")
            ultima_impresion = time.time()

        cv2.imshow("Deteccion", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("e") and piezas:
            x, y, phi = piezas[0][1]
            linea = f"P {x:.1f},{y:.1f},{phi:.1f}\n"
            if ser:
                ser.write(linea.encode())
            print("Enviado:", linea.strip())
        elif k == ord("q"):
            break

    cv2.destroyAllWindows()
    if ser:
        ser.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Detección de piezas en L con ESP32-CAM")
    ap.add_argument("modo", nargs="?", default="detectar", choices=["detectar", "color", "plano"])
    ap.add_argument("--serial", help="puerto del ESP32 del robot, p. ej. COM8")
    ap.add_argument("--camera", default=IP_CAMARA,
                    help="IP o nombre del ESP32-CAM, p. ej. 192.168.20.220")
    args = ap.parse_args()

    if args.modo == "color":
        modo_color(args.camera)
    elif args.modo == "plano":
        modo_plano(args.camera)
    else:
        modo_detectar(args.serial, args.camera)
