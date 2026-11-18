#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials

// ===========================
const char *ssid = "JJ";
const char *password = "elcoditofc";

// true: la cámara CREA su propia red con ese nombre y contraseña (IP fija
//       192.168.4.1). Apagar el hotspot del celular para que la PC no se
//       confunda entre las dos redes con el mismo nombre.
// false: se conecta como cliente a una red existente (hotspot / router).
const bool MODO_AP = true;
// Canal Wi-Fi (1, 6 u 11). En la universidad el 6 está ocupado ~62 % y el 1
// ~80 %; el 11 estaba en ~32 %. Si sigue lento, probar los otros.
const int  CANAL_AP = 11;
// Potencia de transmisión en modo AP. La PC está cerca: con menos potencia la
// placa pide picos de corriente más chicos (menos caídas de voltaje).
// Si la PC está lejos y se corta, subir a WIFI_POWER_15dBm o WIFI_POWER_19_5dBm.
const wifi_power_t POTENCIA_AP = WIFI_POWER_11dBm;

void startCameraServer();
void setupLedFlash();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  // 10 MHz en vez de 20: el reloj de la cámara mete menos ruido en la radio
  // Wi-Fi de la ESP32-CAM (en QVGA sigue dando de sobra cuadros por segundo)
  config.xclk_freq_hz = 10000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    // 10..63: número más alto = JPEG más liviano. 15 se ve casi igual que 10 y
    // pesa bastante menos, así el stream aguanta mejor cuando la señal está floja.
    s->set_quality(s, 15);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  if (MODO_AP) {
    // La cámara crea su propia red (mismo nombre y contraseña de arriba).
    // La PC se conecta directo a ella; la cámara queda siempre en 192.168.4.1.
    WiFi.mode(WIFI_AP);
    // máx. 1 equipo conectado: así el celular u otra PC no le roba ancho de banda
    if (!WiFi.softAP(ssid, password, CANAL_AP, 0, 1)) {
      Serial.println("ERROR: no se pudo crear la red (la contraseña debe tener 8+ caracteres)");
      return;
    }
    WiFi.setSleep(false);
    WiFi.setTxPower(POTENCIA_AP);
    Serial.println("Red Wi-Fi propia creada");
  } else {
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);

    Serial.print("WiFi connecting");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
  }

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(MODO_AP ? WiFi.softAPIP() : WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // El servidor web corre en otra tarea. Aquí solo se reporta el estado del Wi-Fi.
  if (MODO_AP) {
    Serial.printf("Equipos conectados a la red de la camara: %d\n", WiFi.softAPgetStationNum());
  } else {
    //   > -60 dBm buena, -60..-70 aceptable, < -70 dBm el stream se va a trabar.
    Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
  }
  delay(10000);
}
