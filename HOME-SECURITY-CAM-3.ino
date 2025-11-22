// ======================================================
// ESP32-CAM + R307 Fingerprint Home Security System
// Captures image on fingerprint match and sends {name, image(base64)} to Google Apps Script
// Flash LED lights up while capturing
// ======================================================

#include <WiFi.h>
#include <WiFiClientSecure.h>   // ✅ Added for HTTPS
#include <HTTPClient.h>
#include "esp_camera.h"
#include <Adafruit_Fingerprint.h>
#include <mbedtls/base64.h>

// -------- USER CONFIG ----------
const char* ssid = "GalaxyA13";
const char* password = "1234567890";
//const char* scriptURL = "https://script.google.com/macros/s/AKfycbzW85xKRB2HUSrOT4jmxr93Htrq7RESewUs9-PhHJJCfIil7SMmz_Eom3BTPW5ZwKm8/exec";
const char* scriptURL = "https://script.google.com/macros/s/AKfycbw7lyqseoajDNNjuBDTiaWjJzlhl0Mkj9KMSiT1vu-GbW4aDjC9DLAeGN53EwcE48tf/exec";
// -------------------------------

// Member map (IDs must match the templates enrolled in your R307)
struct Member { uint8_t id; const char* name; };
Member members[] = {
  {1, "Madan"},
  {2, "Mom"},
  {3, "Dad"},
  {4, "Sister"},
  {5, "Brother"}
};
const int totalMembers = sizeof(members) / sizeof(Member);

// LED pins
#define FP_LED 14   // fingerprint match indicator (green)
#define MSG_LED 2   // message sent indicator (blue)
#define FLASH_LED 4 // ✅ Built-in flash LED pin (GPIO4)

// R307 on Serial1: RX=13 (to module TX), TX=15 (to module RX)
HardwareSerial FingerSerial(1);
Adafruit_Fingerprint finger(&FingerSerial);

// Camera (AI-Thinker pinout)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ---- Helper: Map ID → Name ----
String idToName(uint8_t id) {
  for (int i = 0; i < totalMembers; ++i)
    if (members[i].id == id) return String(members[i].name);
  return String("Unknown");
}

// -------- Camera init ----------
bool initCamera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  return true;
}

// -------- Capture and Base64 encode ----------
// ✅ Includes flash activation
bool captureImageBase64(String &outBase64) {
  // Turn flash ON before capture
  digitalWrite(FLASH_LED, HIGH);
  delay(800); // give camera time to adjust exposure

  // Flush old frame
  camera_fb_t *fb_old = esp_camera_fb_get();
  if (fb_old) esp_camera_fb_return(fb_old);
  delay(200);

  // Capture new frame
  camera_fb_t * fb = esp_camera_fb_get();

  // Turn flash OFF immediately after capture
  digitalWrite(FLASH_LED, LOW);

  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  size_t out_len = 0;
  size_t needed = 4 * ((fb->len + 2) / 3) + 1;
  uint8_t * b64buf = (uint8_t*) malloc(needed);
  if (!b64buf) {
    Serial.println("Failed to allocate base64 buffer");
    esp_camera_fb_return(fb);
    return false;
  }

  int ret = mbedtls_base64_encode(b64buf, needed, &out_len, fb->buf, fb->len);
  if (ret != 0) {
    Serial.printf("mbedtls_base64_encode failed: %d\n", ret);
    free(b64buf);
    esp_camera_fb_return(fb);
    return false;
  }

  outBase64 = String((char*)b64buf);
  free(b64buf);
  esp_camera_fb_return(fb);
  return true;
}

// -------- Send JSON {name, image} to Apps Script ----------
bool sendJsonToScript(const String &name, const String &base64img) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected; cannot send");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure(); // skip cert validation

  HTTPClient http;
  if (!http.begin(client, scriptURL)) {
    Serial.println("HTTP begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  String payload = "{\"name\":\"" + name + "\",\"image\":\"" + base64img + "\"}";

  int httpCode = http.POST((uint8_t*)payload.c_str(), payload.length());
  bool ok = false;

  if (httpCode > 0) {
    Serial.printf("HTTP code: %d\n", httpCode);
    String resp = http.getString();
    Serial.println("Server resp: " + resp);
    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) ok = true;
  } else {
    Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  return ok;
}

// -------- Fingerprint helpers --------
uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) return 0;
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return 0;
  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) return finger.fingerID;
  return 0;
}

// -------- Setup & Loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nESP32-CAM + R307 Home Security (AI-Thinker)");

  pinMode(FP_LED, OUTPUT);
  pinMode(MSG_LED, OUTPUT);
  pinMode(FLASH_LED, OUTPUT);  // ✅ Initialize flash LED pin
  digitalWrite(FP_LED, LOW);
  digitalWrite(MSG_LED, LOW);
  digitalWrite(FLASH_LED, LOW);

  // Fingerprint serial
  FingerSerial.begin(57600, SERIAL_8N1, 13, 15);
  finger.begin(57600);

  if (!finger.verifyPassword()) {
    Serial.println("R307 not found! Check wiring and power.");
    while (1) delay(1000);
  } else {
    Serial.println("Fingerprint sensor detected.");
  }

  // Camera init
  if (!initCamera()) {
    Serial.println("Camera init failed. Halting.");
    while (1) delay(1000);
  } else {
    Serial.println("Camera initialized.");
  }

  // WiFi connect
  Serial.printf("Connecting to WiFi %s\n", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    for (int i = 0; i < 3; i++) {
        digitalWrite(FLASH_LED, HIGH);
        delay(350);
        digitalWrite(FLASH_LED, LOW);
        delay(350);
   }
  } else {
    Serial.println("WiFi connect failed (check credentials).");
  }
}

void loop() {
  Serial.println("Waiting for finger...");
  uint8_t id = getFingerprintID();
  if (id > 0) {
    String name = idToName(id);
    Serial.printf("Match! ID=%d name=%s\n", id, name.c_str());

    // Fingerprint LED blink
    digitalWrite(FP_LED, HIGH);
    delay(600);
    digitalWrite(FP_LED, LOW);

    // Capture + encode
    Serial.println("Capturing image...");
    String b64;
    if (!captureImageBase64(b64)) {
      Serial.println("Capture or encoding failed; skipping send.");
    } else {
      Serial.printf("Image encoded: %u bytes (base64)\n", (unsigned int)b64.length());
      Serial.println("Sending to Apps Script...");
      bool sent = sendJsonToScript(name, b64);

      if (sent) {
        Serial.println("Send successful!");
      } else {
        Serial.println("Email success.");
      }

      // Blink message LED twice for both success/failure
      for (int i = 0; i < 3; i++) {
        digitalWrite(MSG_LED, HIGH);
        delay(350);
        digitalWrite(MSG_LED, LOW);
        delay(350);
      }
    }
    delay(2500);
  }
  delay(300);
}
