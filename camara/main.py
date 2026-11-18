"""
Detección de piezas en L con ESP32-CAM.
Entrega la posición (x, y) en mm y la rotación phi en grados, en el MARCO DEL
ROBOT (el mismo de la cinemática inversa del ESP32: x adelante, y a la
izquierda, phi antihorario visto desde arriba).

Requisitos:  pip install opencv-python numpy pyserial

Uso (en este orden la primera vez):
  1) python main.py color          -> ajustar el color de la pieza
  2) python main.py plano          -> calibrar píxeles -> mm
  3) python main.py                -> detección en vivo
     python main.py --serial COM8  -> y con la tecla 'e' envía
                                      "P x,y,phi" al robot
  4) python main.py auto --serial COM8 [--vel 50]
        -> modo automático: la banda avanza, al ver una pieza la detiene,
           mide la pose con la pieza quieta, manda "P x,y,phi", espera a
           que el brazo termine y vuelve a encender la banda.
           Teclas: 'p' pausa/continúa, 'c' continúa después de un error,
           'q' sale (apaga la banda).

Convención de phi: ángulo del vector que va de la esquina EXTERIOR de la L a
la esquina INTERIOR (la del ángulo cóncavo), medido desde el eje x del robot.
Es único para cualquier orientación de 0 a 360°.
"""
import argparse
import json
import math
import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

try:
    import msvcrt          # Windows: leer teclas de la consola sin bloquear
except ImportError:
    msvcrt = None

# ========================= CONFIGURACIÓN =========================
IP_CAMARA   = "192.168.4.1"                        # fija con la cámara en modo AP (su propia red)
URL_STREAM  = f"http://{IP_CAMARA}:81/stream"
URL_CAPTURA = f"http://{IP_CAMARA}/capture"          # respaldo si el stream falla
ARCHIVO_CAL = Path(__file__).with_name("calibracion_camara.json")   # junto a este script
AREA_MIN    = 800                            # [px] contornos más pequeños se ignoran
HSV_BAJO    = [0, 0, 180]                   # color inicial (amarillo); se ajusta con "color"
HSV_ALTO    = [179, 50, 255]

# --- Modo automático (banda + brazo) ---
BAUDIOS        = 115200
# Corrección fija [mm] que se suma a la posición medida (marco del robot).
# Usarla solo si el error de agarre es SIEMPRE el mismo en toda la zona.
# Ej.: la garra cae 20 mm antes de la pieza (le falta x) -> (20.0, 0.0).
CORRECCION_XY  = (0.0, 0.0)
VEL_BANDA      = 10          # [%] velocidad de la banda (comando M), el signo da el sentido
                             #     duty real = 80 + (VEL-1)*175/99  ->  30 % ≈ 131/255
# Zona donde se toman las piezas, en mm en el marco del robot:
# (x_min, x_max, y_min, y_max). Debe pquedar dentro del alcance del brazo.
# None = cualquier lugar de la imagen.
ZONA_TOMA      = None
# Alcance del brazo con pitch -90 (IK del firmware). La aproximación sobre la
# pieza baja de Z_ALTO = 100 hasta Z_APROX_MIN = 70 si hace falta, y a 70 mm
# llega hasta 239 mm. Se deja margen para que la pieza no quede al límite.
ALCANCE_MIN    = 60.0        # [mm] distancia mínima del centro de la base
ALCANCE_MAX    = 217.0       # [mm] distancia máxima para frenar la banda (con zToma=70 la aproximación queda a z>=90)
MARGEN_MEDIR   = 5.0         # [mm] al medir se acepta hasta ALCANCE_MAX + esto (límite real 223 con z=90)
DESTINO_XY     = (82.1, 225.5)    # igual que XD, YD del firmware (R_D=240, ANG_D=70)
RADIO_DESTINO  = 70.0        # [mm] se ignoran piezas cerca del destino (las ya colocadas)
ESPERA_FRENADO = 0.8         # [s] después de M0, antes de empezar a medir
N_ESTABLE      = 5           # lecturas seguidas casi iguales para aceptar la pose
TOL_MM         = 3.0         # [mm]   variación máxima entre esas lecturas
TOL_GRADOS     = 3.0         # [°]
TIMEOUT_MEDIR  = 5.0         # [s] si no se logra una pose estable -> error
TIMEOUT_CICLO  = 90.0        # [s] si el brazo no responde -> error
# =================================================================


# ------------------------------ Cámara ------------------------------
class Camara:
    """Lee el stream en un hilo aparte y se queda SOLO con el último cuadro.
    Sin esto, OpenCV acumula cuadros en un buffer y, como el procesamiento es
    más lento que el stream, se trabaja con imágenes de 1-2 s de atraso: la
    banda se frena cuando la pieza ya se salió del encuadre."""

    def __init__(self, ip_camara=IP_CAMARA):
        base_url = f"http://{ip_camara.strip().rstrip('/') }"
        self.url_stream = f"{base_url}:81/stream"
        self.url_captura = f"{base_url}/capture"
        self._frame = None
        self._id = 0            # contador de cuadros recibidos
        self._id_leido = 0      # último cuadro entregado por leer()
        self._cond = threading.Condition()
        self.cap = self._abrir_stream()
        self.usar_stream = self.cap is not None
        # El hilo corre siempre: si el stream se cae, lo vuelve a abrir solo
        # (mientras tanto leer() usa /capture)
        threading.Thread(target=self._hilo_stream, daemon=True).start()
        if not self.usar_stream:
            try:
                urllib.request.urlopen(self.url_captura, timeout=3).close()
            except Exception:
                print(f"\nERROR: no hay respuesta de la cámara en {base_url}")
                print("  - ¿La ESP32-CAM está encendida y conectada al Wi-Fi? Mira su Monitor")
                print("    Serie (115200): al arrancar imprime la IP con la que quedó.")
                print("  - Si la IP cambió, usa --camera <ip> o cambia IP_CAMARA en main.py.")
                print("  - La PC tiene que estar en la misma red Wi-Fi que la cámara.")
                raise SystemExit(1)
            print("Stream no disponible, se usará /capture (más lento).")

    def _abrir_stream(self):
        """Abre el stream con timeouts cortos (por defecto FFmpeg espera ~30 s
        cuando el Wi-Fi se traba). Devuelve el VideoCapture o None."""
        cap = cv2.VideoCapture(self.url_stream, cv2.CAP_FFMPEG,
                               [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000,
                                cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000])
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _hilo_stream(self):
        fallos = 0
        cuadros, t_fps = 0, time.time()
        while True:
            if self.cap is None:                     # caído: reintentar
                time.sleep(1.0)
                self.cap = self._abrir_stream()
                if self.cap is not None:
                    print("Stream recuperado.")
                    self.usar_stream = True
                    fallos = 0
                continue

            ok, frame = self.cap.read()
            if not ok:
                fallos += 1
                if fallos > 10:
                    print("Stream perdido, reconectando (mientras tanto /capture, más lento)...")
                    self.usar_stream = False
                    self.cap.release()
                    self.cap = None
                time.sleep(0.02)
                continue
            fallos = 0
            with self._cond:
                self._frame = frame
                self._id += 1
                self._cond.notify_all()

            # Aviso si el stream va lento (casi siempre es señal Wi-Fi débil)
            cuadros += 1
            dt = time.time() - t_fps
            if dt >= 5.0:
                fps = cuadros / dt
                if fps < 8:
                    print(f"AVISO: stream lento, {fps:.1f} cuadros/s (revisa la señal Wi-Fi de la cámara)")
                cuadros, t_fps = 0, time.time()

    def leer(self):
        """Devuelve el cuadro más reciente que todavía no se haya entregado
        (espera hasta 1 s a que llegue uno nuevo)."""
        if self.usar_stream:
            with self._cond:
                self._cond.wait_for(lambda: self._id != self._id_leido, timeout=1.0)
                if self._id != self._id_leido:
                    self._id_leido = self._id
                    return self._frame
            return None
        try:
            with urllib.request.urlopen(self.url_captura, timeout=3) as r:
                datos = np.frombuffer(r.read(), np.uint8)
            return cv2.imdecode(datos, cv2.IMREAD_COLOR)
        except Exception as e:
            print("Error leyendo la cámara:", e)
            time.sleep(0.5)
            return None


# ----------------------------- Teclado ------------------------------
def tecla(ms=1):
    """Tecla presionada en la ventana de OpenCV O en la consola (código en
    minúscula), o -1 si no hay ninguna. cv2.waitKey solo ve las teclas cuando
    la ventana de video tiene el foco; así también funcionan desde la consola."""
    k = cv2.waitKey(ms) & 0xFF
    if k != 255:
        return ord(chr(k).lower())
    while msvcrt is not None and msvcrt.kbhit():
        ch = msvcrt.getwch()
        if ch in ("\x00", "\xe0"):      # flechas / teclas de función: 2 códigos
            msvcrt.getwch()
            continue
        return ord(ch.lower())
    return -1


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
    cx += CORRECCION_XY[0]
    cy += CORRECCION_XY[1]
    ext, inte = a_mundo(H, np.array(esquinas))
    phi = 0#math.degrees(math.atan2(inte[1] - ext[1], inte[0] - ext[0]))
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
        k = tecla(1)
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
    confirmar = False       # 2.º 'c' guarda aunque haya puntos con error grande
    print("Haz clic en un punto de referencia y escribe sus coordenadas en la consola.")
    print("Marco del ROBOT: x hacia adelante, y hacia la IZQUIERDA del robot (mm).")
    print("Usa 6 o más puntos repartidos, que no estén en línea.")
    print("'u' deshace el último punto, 'c' calcula y guarda, 'q' sale sin guardar.")
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
        if k == ord("u") and pts_img:
            print(f"Deshecho el punto {pts_img.pop()} -> {pts_mm.pop()}")
        if k == ord("c"):
            if len(pts_img) < 4:
                print(f"Faltan puntos: hay {len(pts_img)}, se necesitan al menos 4.")
                continue
            H, _ = cv2.findHomography(np.float32(pts_img), np.float32(pts_mm))
            if H is None:
                print("No se pudo calcular: revisa que los puntos no estén en línea.")
                continue
            # Error de cada punto: con 6+ puntos un número mal escrito se nota acá
            est = a_mundo(H, pts_img)
            print("\nError de cada punto (calibración vs lo que escribiste):")
            peor = 0.0
            for (u, v), (xm, ym), (xe, ye) in zip(pts_img, pts_mm, est):
                e = math.hypot(xe - xm, ye - ym)
                peor = max(peor, e)
                marca = "  <-- revisar" if e > 5 else ""
                print(f"  píxel ({u:3},{v:3})  escrito ({xm:6.1f},{ym:6.1f})  "
                      f"calculado ({xe:6.1f},{ye:6.1f})  error {e:4.1f} mm{marca}")
            if len(pts_img) == 4:
                print("  (con solo 4 puntos el error siempre da 0: no se puede verificar)")
            elif peor > 5:
                print("  Hay puntos con más de 5 mm de error: 'u' para quitarlos y volver a marcar,")
                print("  o 'c' otra vez para guardar igual.")
                if not confirmar:
                    confirmar = True
                    continue
            cal["H"] = H.tolist()
            guardar_cal(cal)
            break
        if k == ord("q"):
            break
    cv2.destroyAllWindows()


# --------------------------- Robot (serial) ---------------------------
class Robot:
    """Comunicación con el ESP32 del brazo sin bloquear el video."""

    def __init__(self, puerto):
        import serial
        from serial.tools import list_ports
        try:
            self.ser = serial.Serial(puerto, BAUDIOS, timeout=0)
        except serial.SerialException as e:
            print(f"\nNo se pudo abrir {puerto}: {e}")
            if "denegado" in str(e).lower() or "denied" in str(e).lower():
                print("  -> El puerto está ocupado: cierra el Monitor Serie del Arduino IDE /")
                print("     VS Code u otra terminal que tenga el script corriendo.")
            print("Puertos disponibles:")
            for p in list_ports.comports():
                print(f"  {p.device:6}  {p.description}")
            raise SystemExit(1)
        self.buf = b""

    def enviar(self, linea):
        self.ser.write((linea + "\n").encode())
        print(">>", linea)

    def lineas(self):
        """Devuelve las líneas completas que hayan llegado (puede ser [])."""
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        salida = []
        while b"\n" in self.buf:
            linea, self.buf = self.buf.split(b"\n", 1)
            linea = linea.decode(errors="replace").strip()
            if linea:
                print("<<", linea)
                salida.append(linea)
        return salida

    def esperar_listo(self, timeout=6.0):
        """La ESP32 se reinicia al abrir el puerto; espera su 'OK listo'."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if any(l.startswith("OK listo") for l in self.lineas()):
                return True
            time.sleep(0.05)
        return False

    def verificar_firmware(self, timeout=2.0):
        """Manda '?' y espera 'ESTADO ...' (solo lo responde firmware-brazo.ino)."""
        self.enviar("?")
        t0 = time.time()
        while time.time() - t0 < timeout:
            if any(l.startswith("ESTADO") for l in self.lineas()):
                return True
            time.sleep(0.05)
        return False

    def cerrar(self):
        self.ser.close()


# --------------------------- Detección -------------------------------
def detectar(frame, bajo, alto, H):
    """Busca piezas en L, las dibuja sobre el frame y devuelve
    [(area, (x, y, phi)), ...] de la más grande a la más chica."""
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
    return piezas


# --------------------------- Modo: detección --------------------------
def modo_detectar(puerto_serial, ip_camara):
    cal = cargar_cal()
    bajo = cal.get("hsv_bajo", HSV_BAJO)
    alto = cal.get("hsv_alto", HSV_ALTO)
    H = np.array(cal["H"]) if "H" in cal else None
    unidad = "mm" if H is not None else "px"
    if H is None:
        print("AVISO: no hay calibración de plano; se reporta en píxeles.")

    robot = None
    if puerto_serial:
        robot = Robot(puerto_serial)
        robot.esperar_listo()

    cam = Camara(ip_camara)
    ultima_impresion = 0
    print("'e' envía la pose de la pieza más grande, 'q' sale.")
    while True:
        frame = cam.leer()
        if robot:
            robot.lineas()                     # muestra lo que responde el brazo
        if frame is None:
            continue
        piezas = detectar(frame, bajo, alto, H)

        if piezas and time.time() - ultima_impresion > 0.5:
            x, y, phi = piezas[0][1]
            print(f"Pieza: x={x:7.1f} {unidad}  y={y:7.1f} {unidad}  phi={phi:6.1f}°")
            ultima_impresion = time.time()

        cv2.imshow("Deteccion", frame)
        k = tecla(1)
        if k == ord("e") and piezas:
            x, y, phi = piezas[0][1]
            linea = f"P {x:.1f},{y:.1f},{phi:.1f}"
            if robot:
                robot.enviar(linea)
            else:
                print("Pose:", linea)
        elif k == ord("q"):
            break

    cv2.destroyAllWindows()
    if robot:
        robot.cerrar()


# --------------------------- Modo: automático -------------------------
def dif_angulo(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def en_zona(x, y, margen=0.0):
    if math.hypot(x - DESTINO_XY[0], y - DESTINO_XY[1]) < RADIO_DESTINO:
        return False                            # pieza ya colocada en la plantilla
    if not ALCANCE_MIN <= math.hypot(x, y) <= ALCANCE_MAX + margen:
        return False                            # el brazo no llega
    if ZONA_TOMA is None:
        return True
    x0, x1, y0, y1 = ZONA_TOMA
    return x0 <= x <= x1 and y0 <= y <= y1


def motivo_zona(x, y):
    """Por qué en_zona() rechaza una pieza (texto corto para la pantalla)."""
    if math.hypot(x - DESTINO_XY[0], y - DESTINO_XY[1]) < RADIO_DESTINO:
        return "en destino"
    d = math.hypot(x, y)
    if d > ALCANCE_MAX:
        return f"lejos d={d:.0f}>{ALCANCE_MAX:.0f}"
    if d < ALCANCE_MIN:
        return f"cerca d={d:.0f}<{ALCANCE_MIN:.0f}"
    return "fuera de ZONA_TOMA"


def dispersion(lecturas):
    """Máxima distancia [mm] y máxima diferencia de ángulo [°] respecto al promedio."""
    xm, ym, pm = pose_promedio(lecturas)
    d = max(math.hypot(x - xm, y - ym) for x, y, _ in lecturas)
    a = max(abs(dif_angulo(p, pm)) for _, _, p in lecturas)
    return d, a


def pose_estable(lecturas):
    d, a = dispersion(lecturas)
    return d <= TOL_MM and a <= TOL_GRADOS


def motivo_medicion(lecturas, ultima_vista):
    """Explica por qué no se logró una pose estable (para ajustar parámetros)."""
    if not lecturas:
        if ultima_vista is None:
            return "no se detectó ninguna pieza en L (revisa el color: python main.py color)"
        x, y, phi = ultima_vista
        return (f"la pieza quedó fuera del alcance: x={x:.0f} y={y:.0f} "
                f"(distancia {math.hypot(x, y):.0f} mm, máximo {ALCANCE_MAX + MARGEN_MEDIR:.0f})")
    if len(lecturas) < N_ESTABLE:
        return f"la pieza solo se detectó {len(lecturas)} de {N_ESTABLE} veces (detección intermitente)"
    d, a = dispersion(lecturas)
    return (f"la pose varía ±{d:.1f} mm / ±{a:.1f}° entre cuadros "
            f"(tolerancia {TOL_MM} mm / {TOL_GRADOS}°); súbelas o mejora la iluminación")


def pose_promedio(lecturas):
    x = float(np.mean([l[0] for l in lecturas]))
    y = float(np.mean([l[1] for l in lecturas]))
    phi = math.degrees(math.atan2(np.mean([math.sin(math.radians(l[2])) for l in lecturas]),
                                  np.mean([math.cos(math.radians(l[2])) for l in lecturas])))
    return x, y, phi


def dibujar_zonas(frame, Hinv):
    if ZONA_TOMA is not None:
        x0, x1, y0, y1 = ZONA_TOMA
        esq = a_mundo(Hinv, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]).astype(np.int32)
        cv2.polylines(frame, [esq], True, (0, 255, 255), 2)          # amarillo: zona de toma
    # verde: límite de alcance del brazo (arco de ALCANCE_MAX alrededor de la base)
    a = np.radians(np.linspace(-90, 90, 61))
    arco = np.c_[ALCANCE_MAX * np.cos(a), ALCANCE_MAX * np.sin(a)]
    cv2.polylines(frame, [a_mundo(Hinv, arco).astype(np.int32)], False, (0, 255, 0), 1)
    ang = np.linspace(0, 2 * np.pi, 32, endpoint=False)
    circ = np.c_[DESTINO_XY[0] + RADIO_DESTINO * np.cos(ang),
                 DESTINO_XY[1] + RADIO_DESTINO * np.sin(ang)]
    cv2.polylines(frame, [a_mundo(Hinv, circ).astype(np.int32)], True, (0, 0, 255), 1)


def modo_auto(puerto_serial, ip_camara, vel):
    cal = cargar_cal()
    if "H" not in cal:
        print("ERROR: falta la calibración del plano. Corre primero: python main.py plano")
        return
    if not puerto_serial:
        print("ERROR: el modo auto necesita --serial COMx")
        return
    bajo = cal.get("hsv_bajo", HSV_BAJO)
    alto = cal.get("hsv_alto", HSV_ALTO)
    H = np.array(cal["H"])
    Hinv = np.linalg.inv(H)

    robot = Robot(puerto_serial)
    print("Esperando al robot...")
    if not robot.esperar_listo():
        print("AVISO: no llegó 'OK listo', se continúa de todas formas.")
    if not robot.verificar_firmware():
        print("\nERROR: el ESP32 no respondió como firmware-brazo.ino.")
        print("Carga firmware-brazo/firmware-brazo.ino en la placa y vuelve a intentar.")
        robot.cerrar()
        return
    cam = Camara(ip_camara)

    estado, t_estado = "BANDA", time.time()
    lecturas = []
    ultima_vista = None
    pose_disparo = (0.0, 0.0, 0.0)
    ultimo_aviso = 0.0
    piezas_ok = 0

    def cambiar(nuevo):
        nonlocal estado, t_estado
        estado, t_estado = nuevo, time.time()
        print(f"[{nuevo}]")

    def error(motivo):
        robot.enviar("M0")
        print("ERROR:", motivo)
        print("Retira la pieza si hace falta y presiona 'c' para continuar.")
        cambiar("ERROR")

    robot.enviar(f"M{vel}")
    print(f"[{estado}]  'p' pausa, 'c' continúa tras error, 'q' sale.")
    while True:
        frame = cam.leer()
        lineas = robot.lineas()
        if frame is None:
            continue
        todas = detectar(frame, bajo, alto, H)
        piezas = [p for p in todas if en_zona(*p[1][:2])]                  # para frenar
        piezas_med = [p for p in todas if en_zona(*p[1][:2], MARGEN_MEDIR)]  # para medir
        dibujar_zonas(frame, Hinv)
        dt = time.time() - t_estado

        # Diagnóstico: piezas en L vistas pero descartadas por la zona
        for _, (x, y, _) in todas:
            if not en_zona(x, y):
                u, v = a_mundo(Hinv, [(x, y)])[0].astype(int)
                cv2.putText(frame, motivo_zona(x, y), (int(u) - 40, int(v) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 2)
        if estado == "BANDA" and todas and not piezas and time.time() - ultimo_aviso > 1.0:
            x, y, _ = todas[0][1]
            print(f"    pieza vista pero descartada ({motivo_zona(x, y)}): x={x:.0f} y={y:.0f}")
            ultimo_aviso = time.time()

        if estado == "BANDA":
            if piezas:                              # apareció una pieza: parar la banda
                robot.enviar("M0")
                pose_disparo = piezas[0][1]
                lecturas = []
                ultima_vista = None
                cambiar("FRENANDO")

        elif estado == "FRENANDO":
            if dt >= ESPERA_FRENADO:
                cambiar("MIDIENDO")

        elif estado == "MIDIENDO":
            # Un cuadro sin detección no reinicia la cuenta: se usan las
            # últimas N lecturas válidas (la pieza está quieta).
            if todas:
                ultima_vista = todas[0][1]
            if piezas_med:
                lecturas = (lecturas + [piezas_med[0][1]])[-N_ESTABLE:]
            if len(lecturas) == N_ESTABLE and pose_estable(lecturas):
                x, y, phi = pose_promedio(lecturas)
                print(f"    recorrido después de M0: "
                      f"{math.hypot(x - pose_disparo[0], y - pose_disparo[1]):.0f} mm")
                robot.enviar(f"P {x:.1f},{y:.1f},{phi:.1f}")
                cambiar("CICLO")
            elif dt > TIMEOUT_MEDIR:
                error("no se logró una pose estable: " + motivo_medicion(lecturas, ultima_vista))
                for l in lecturas:
                    print(f"    x={l[0]:7.1f}  y={l[1]:7.1f}  phi={l[2]:7.1f}")

        elif estado == "CICLO":
            if any(l.startswith("OK ciclo completo") for l in lineas):
                piezas_ok += 1
                robot.enviar(f"M{vel}")
                cambiar("BANDA")
            elif any(l.startswith("ERR") for l in lineas):
                error("el brazo no pudo completar el ciclo")
            elif dt > TIMEOUT_CICLO:
                error("el brazo no respondió a tiempo")

        color = (0, 0, 255) if estado == "ERROR" else (255, 255, 0)
        cv2.putText(frame, f"{estado}  piezas={piezas_ok}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow("Automatico", frame)

        k = tecla(1)
        if k == ord("q"):
            break
        elif k == ord("p"):
            if estado == "PAUSA":
                robot.enviar(f"M{vel}")
                cambiar("BANDA")
            elif estado in ("BANDA", "FRENANDO", "MIDIENDO"):
                robot.enviar("M0")
                cambiar("PAUSA")
        elif k == ord("c") and estado == "ERROR":
            robot.enviar(f"M{vel}")
            cambiar("BANDA")

    robot.enviar("M0")
    time.sleep(0.2)
    cv2.destroyAllWindows()
    robot.cerrar()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Detección de piezas en L con ESP32-CAM")
    ap.add_argument("modo", nargs="?", default="detectar",
                    choices=["detectar", "color", "plano", "auto"])
    ap.add_argument("--serial", help="puerto del ESP32 del robot, p. ej. COM8")
    ap.add_argument("--camera", default=IP_CAMARA,
                    help="IP o nombre del ESP32-CAM, p. ej. 192.168.20.220")
    ap.add_argument("--vel", type=int, default=VEL_BANDA,
                    help="velocidad de la banda en modo auto, -100..100 (%%)")
    args = ap.parse_args()

    if args.modo == "color":
        modo_color(args.camera)
    elif args.modo == "plano":
        modo_plano(args.camera)
    elif args.modo == "auto":
        modo_auto(args.serial, args.camera, max(-100, min(100, args.vel)))
    else:
        modo_detectar(args.serial, args.camera)
