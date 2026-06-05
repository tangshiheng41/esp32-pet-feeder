#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_camera.h>
#include <img_converters.h>
#include <pet_feeder_inferencing.h>
#include <HX711.h>
#include <ESP32Servo.h>
#include "secrets.h"

// Pins (your current wiring)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    -1
#define SIOD_GPIO_NUM    5
#define SIOC_GPIO_NUM    4
#define Y9_GPIO_NUM      47
#define Y8_GPIO_NUM      42
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      11
#define Y5_GPIO_NUM      41
#define Y4_GPIO_NUM      10
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM      8
#define VSYNC_GPIO_NUM   39
#define HREF_GPIO_NUM    40
#define PCLK_GPIO_NUM    21

static const char *FW_TAG = "WEB_TAG_SERVO_V2";
static const uint32_t INFER_EVERY_MS = 1300;
static const uint32_t WEIGHT_READ_EVERY_MS = 500;
static const uint32_t FEED_STATE_TICK_MS = 50;
static const uint32_t SERVO_TEST_OPEN_MS = 1000;
static const char *TARGET_LABEL = "pet front";
static const float CONFIDENCE_THRESHOLD = 0.78f;
static const float MIN_VALID_CONFIDENCE = 0.70f; // below this, show "unknown" instead of forcing a class
static const float HX711_CALIBRATION_FACTOR = 430.0f;

// Feeding parameters
static const float LOW_FOOD_THRESHOLD = 20.0f;     // g
static const float FULL_FOOD_THRESHOLD = 500.0f;   // g
static const float TARGET_FEED_WEIGHT = 100.0f;    // g
static const uint32_t FEED_TIMEOUT_MS = 8000;
static const uint32_t FEED_COOLDOWN_MS = 15000;

#define HX711_DT_PIN  1
#define HX711_SCK_PIN 2
#define SERVO_PIN     18
static const int SERVO_CLOSED_ANGLE = 0;
static const int SERVO_OPEN_ANGLE = 90;
static const bool SERVO_REVERSED = true;

static int servo_apply_angle(int angle) {
  if (!SERVO_REVERSED) return angle;
  return 180 - angle;
}

static WebServer server(80);
static camera_config_t cam_cfg;

static uint8_t *snapshot_buf = nullptr;   // EI gray input
static size_t snapshot_buf_size = 0;
static uint8_t *rgb888_buf = nullptr;     // JPEG decode buffer
static size_t rgb888_buf_size = 0;
static HX711 g_scale;
static Servo g_servo;
static String g_lastLabel = "waiting";
static float g_lastConfidence = 0.0f;
static bool g_lastPetDetected = false;
static uint32_t g_lastInferMs = 0;
static uint32_t g_nextInferAt = 0;
static uint32_t g_nextWeightAt = 0;
static uint8_t g_decodeFailStreak = 0;
static float g_currentWeight = -1.0f;
static bool g_weightValid = false;
static bool g_servoTestActive = false;
static uint32_t g_servoCloseAt = 0;
static bool g_cameraReady = false;
static bool g_hx711Ready = false;
static bool g_servoReady = false;
static uint32_t g_nextFeedStateAt = 0;

enum FeedStateType {
  FEED_IDLE = 0,
  FEED_FEEDING,
  FEED_COOLDOWN
};

static FeedStateType g_feedState = FEED_IDLE;
static uint32_t g_feedStartAt = 0;
static uint32_t g_cooldownEndAt = 0;

static const char *feed_state_text(int s) {
  if (s == FEED_FEEDING) return "FEEDING";
  if (s == FEED_COOLDOWN) return "COOLDOWN";
  return "IDLE";
}

static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  if (!snapshot_buf || (offset + length > snapshot_buf_size)) return -1;
  for (size_t i = 0; i < length; i++) out_ptr[i] = (float)snapshot_buf[offset + i];
  return 0;
}

static bool resize_rgb888_to_gray_nn(const uint8_t *src_rgb888, uint32_t src_w, uint32_t src_h,
                                     uint8_t *dst_gray, uint32_t dst_w, uint32_t dst_h) {
  if (!src_rgb888 || !dst_gray || src_w == 0 || src_h == 0 || dst_w == 0 || dst_h == 0) return false;
  for (uint32_t y = 0; y < dst_h; y++) {
    uint32_t src_y = (uint64_t)y * src_h / dst_h;
    for (uint32_t x = 0; x < dst_w; x++) {
      uint32_t src_x = (uint64_t)x * src_w / dst_w;
      size_t idx = (((size_t)src_y * src_w) + src_x) * 3;
      uint8_t r = src_rgb888[idx + 0];
      uint8_t g = src_rgb888[idx + 1];
      uint8_t b = src_rgb888[idx + 2];
      dst_gray[(size_t)y * dst_w + x] = (uint8_t)((30 * r + 59 * g + 11 * b) / 100);
    }
  }
  return true;
}

static bool init_camera() {
  memset(&cam_cfg, 0, sizeof(cam_cfg));
  cam_cfg.ledc_channel = LEDC_CHANNEL_0;
  cam_cfg.ledc_timer = LEDC_TIMER_0;
  cam_cfg.pin_d0 = Y2_GPIO_NUM;
  cam_cfg.pin_d1 = Y3_GPIO_NUM;
  cam_cfg.pin_d2 = Y4_GPIO_NUM;
  cam_cfg.pin_d3 = Y5_GPIO_NUM;
  cam_cfg.pin_d4 = Y6_GPIO_NUM;
  cam_cfg.pin_d5 = Y7_GPIO_NUM;
  cam_cfg.pin_d6 = Y8_GPIO_NUM;
  cam_cfg.pin_d7 = Y9_GPIO_NUM;
  cam_cfg.pin_xclk = XCLK_GPIO_NUM;
  cam_cfg.pin_pclk = PCLK_GPIO_NUM;
  cam_cfg.pin_vsync = VSYNC_GPIO_NUM;
  cam_cfg.pin_href = HREF_GPIO_NUM;
  cam_cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cam_cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cam_cfg.pin_pwdn = PWDN_GPIO_NUM;
  cam_cfg.pin_reset = RESET_GPIO_NUM;
  cam_cfg.xclk_freq_hz = 8000000;
  // This board is most stable in JPEG mode.
  cam_cfg.pixel_format = PIXFORMAT_JPEG;
  cam_cfg.frame_size = FRAMESIZE_QQVGA; // 160x120
  cam_cfg.fb_count = 1;
  cam_cfg.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  cam_cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  cam_cfg.jpeg_quality = 24;

  Serial.println("[CAM] init...");
  if (esp_camera_init(&cam_cfg) != ESP_OK) {
    Serial.println("[CAM] init failed.");
    return false;
  }

  snapshot_buf_size = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
  snapshot_buf = (uint8_t *)ps_malloc(snapshot_buf_size);
  if (!snapshot_buf) snapshot_buf = (uint8_t *)malloc(snapshot_buf_size);
  if (!snapshot_buf) {
    Serial.println("[ERR] snapshot buffer alloc failed.");
    return false;
  }
  memset(snapshot_buf, 0, snapshot_buf_size);

  rgb888_buf_size = (size_t)160 * 120 * 3;
  rgb888_buf = (uint8_t *)ps_malloc(rgb888_buf_size);
  if (!rgb888_buf) rgb888_buf = (uint8_t *)malloc(rgb888_buf_size);
  if (!rgb888_buf) {
    Serial.println("[ERR] rgb888 buffer alloc failed.");
    return false;
  }

  run_classifier_init();
  Serial.println("[CAM] init ok.");
  g_cameraReady = true;
  return true;
}

static bool init_hx711() {
  g_scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
  uint32_t start = millis();
  while (!g_scale.is_ready()) {
    if (millis() - start > 3000) {
      Serial.println("[HX711] not ready on init.");
      g_weightValid = false;
      g_currentWeight = -1.0f;
      g_hx711Ready = false;
      return false;
    }
    delay(50);
  }

  g_scale.set_scale(HX711_CALIBRATION_FACTOR);
  g_scale.tare(10);
  g_weightValid = true;
  g_currentWeight = 0.0f;
  g_hx711Ready = true;
  Serial.println("[HX711] init ok (tare done).");
  return true;
}

static void update_weight_nonblocking() {
  if (!g_scale.is_ready()) {
    g_weightValid = false;
    g_currentWeight = -1.0f;
    Serial.println("[HX711] read error: sensor not ready.");
    return;
  }

  // Single sample keeps read short; called every 500ms.
  float w = g_scale.get_units(1);
  if (!isfinite(w)) {
    g_weightValid = false;
    g_currentWeight = -1.0f;
    Serial.println("[HX711] read error: invalid value.");
    return;
  }
  g_currentWeight = w;
  g_weightValid = true;
}

static bool init_servo() {
  g_servo.setPeriodHertz(50);
  if (!g_servo.attach(SERVO_PIN, 500, 2500)) {
    Serial.println("[SERVO] attach failed.");
    g_servoReady = false;
    return false;
  }
  g_servo.write(servo_apply_angle(SERVO_CLOSED_ANGLE));
  g_servoReady = true;
  Serial.println("[SERVO] init ok (closed).");
  return true;
}

static void trigger_servo_test() {
  if (g_feedState == FEED_FEEDING) return;
  g_servo.write(servo_apply_angle(SERVO_OPEN_ANGLE));
  g_servoTestActive = true;
  g_servoCloseAt = millis() + SERVO_TEST_OPEN_MS;
}

static void start_feeding() {
  g_servo.write(servo_apply_angle(SERVO_OPEN_ANGLE));
  g_feedState = FEED_FEEDING;
  g_feedStartAt = millis();
}

static void stop_feeding_and_cooldown() {
  g_servo.write(servo_apply_angle(SERVO_CLOSED_ANGLE));
  g_feedState = FEED_COOLDOWN;
  g_cooldownEndAt = millis() + FEED_COOLDOWN_MS;
}

static void run_feed_state_machine() {
  if (!g_servoReady || !g_weightValid) return;

  uint32_t now = millis();
  switch (g_feedState) {
    case FEED_IDLE: {
      bool lowFood = (g_currentWeight < LOW_FOOD_THRESHOLD);
      bool notOverflow = (g_currentWeight < FULL_FOOD_THRESHOLD);
      bool petReady = g_lastPetDetected;
      if (lowFood && notOverflow && petReady) {
        start_feeding();
      }
      break;
    }
    case FEED_FEEDING: {
      bool reachedTarget = (g_currentWeight >= TARGET_FEED_WEIGHT);
      bool timeout = (now - g_feedStartAt >= FEED_TIMEOUT_MS);
      if (reachedTarget || timeout) {
        stop_feeding_and_cooldown();
      }
      break;
    }
    case FEED_COOLDOWN: {
      if ((int32_t)(now - g_cooldownEndAt) >= 0) {
        g_feedState = FEED_IDLE;
      }
      break;
    }
  }
}

static bool reinit_camera_driver() {
  esp_camera_deinit();
  delay(60);
  if (esp_camera_init(&cam_cfg) != ESP_OK) {
    Serial.println("[CAM] reinit failed.");
    return false;
  }
  Serial.println("[CAM] reinit ok.");
  return true;
}

static void run_inference_once() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    g_lastLabel = "no_frame";
    g_lastConfidence = 0.0f;
    g_lastPetDetected = false;
    g_lastInferMs = millis();
    return;
  }

  bool ok = false;
  if (fb->format == PIXFORMAT_JPEG && fb->width <= 160 && fb->height <= 120) {
    if (fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb888_buf)) {
      ok = resize_rgb888_to_gray_nn(
          rgb888_buf, fb->width, fb->height, snapshot_buf,
          EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    }
  }
  esp_camera_fb_return(fb);

  if (!ok) {
    g_lastLabel = "decode_err";
    g_lastConfidence = 0.0f;
    g_lastPetDetected = false;
    g_lastInferMs = millis();
    if (g_decodeFailStreak < 255) g_decodeFailStreak++;
    if (g_decodeFailStreak >= 3) {
      g_decodeFailStreak = 0;
      reinit_camera_driver();
    }
    return;
  }
  g_decodeFailStreak = 0;

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = &raw_feature_get_data;

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR ei_err = run_classifier(&signal, &result, false);
  if (ei_err != EI_IMPULSE_OK) {
    g_lastLabel = "ei_err";
    g_lastConfidence = 0.0f;
    g_lastPetDetected = false;
    g_lastInferMs = millis();
    return;
  }

  float best = 0.0f;
  String bestLabel = "none";
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float v = result.classification[i].value;
    const char *label = result.classification[i].label;
    if (v > best) {
      best = v;
      bestLabel = String(label);
    }
  }

  if (best < MIN_VALID_CONFIDENCE) {
    g_lastLabel = "unknown";
    g_lastConfidence = best;
    g_lastPetDetected = false;
  } else {
    g_lastLabel = bestLabel;
    g_lastConfidence = best;
    g_lastPetDetected = (bestLabel.equalsIgnoreCase(TARGET_LABEL) && best >= CONFIDENCE_THRESHOLD);
  }
  g_lastInferMs = millis();
}

static void handle_metrics() {
  char buf[300];
  float out_weight = g_weightValid ? g_currentWeight : -1.0f;
  uint32_t now = millis();
  uint32_t cooldownRemaining = 0;
  if (g_feedState == FEED_COOLDOWN && g_cooldownEndAt > now) {
    cooldownRemaining = (g_cooldownEndAt - now + 999) / 1000;
  }
  snprintf(buf, sizeof(buf),
           "{\"label\":\"%s\",\"confidence\":%.6f,\"petDetected\":%s,\"weight_g\":%.1f,\"feedState\":\"%s\",\"cooldownRemaining\":%lu,\"ts\":%lu}",
           g_lastLabel.c_str(), g_lastConfidence, g_lastPetDetected ? "true" : "false",
           out_weight, feed_state_text(g_feedState), (unsigned long)cooldownRemaining, (unsigned long)g_lastInferMs);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

static void handle_test_servo() {
  if (g_feedState == FEED_FEEDING) {
    server.send(409, "application/json", "{\"ok\":false,\"message\":\"feeding_in_progress\"}");
    return;
  }
  trigger_servo_test();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"servo_test_started\"}");
}

static void handle_root() {
  String html =
      "<!doctype html><html><head><meta charset='utf-8'/>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
      "<title>Pet Tag Confidence (Servo)</title>"
      "<style>body{font-family:Arial;margin:16px;line-height:1.6;}</style>"
      "</head><body>"
      "<h2>Pet Tag + Confidence</h2>"
      "<p><b>Label:</b> <span id='label'>-</span></p>"
      "<p><b>Confidence:</b> <span id='conf'>-</span></p>"
      "<p><b>Result:</b> <span id='result'>-</span></p>"
      "<p><b>Weight:</b> <span id='weight'>-</span> g</p>"
      "<p><b>Feed State:</b> <span id='feedState'>-</span></p>"
      "<p><b>Cooldown:</b> <span id='cooldown'>0</span> s</p>"
      "<p><button id='btnServo' type='button'>Test Servo</button></p>"
      "<p><b>Updated:</b> <span id='ts'>-</span></p>"
      "<script>"
      "document.getElementById('btnServo').addEventListener('click',async()=>{try{await fetch('/testServo?t='+Date.now());}catch(e){}});"
      "async function poll(){"
      "try{const r=await fetch('/metrics?t='+Date.now());const j=await r.json();"
      "document.getElementById('label').textContent=j.label;"
      "document.getElementById('conf').textContent=Number(j.confidence).toFixed(3);"
      "document.getElementById('result').textContent=j.petDetected?'PET_DETECTED':'NOT_TARGET';"
      "document.getElementById('weight').textContent=(Number(j.weight_g)>=0)?Number(j.weight_g).toFixed(1):'-1.0';"
      "document.getElementById('feedState').textContent=j.feedState;"
      "document.getElementById('cooldown').textContent=Number(j.cooldownRemaining||0);"
      "document.getElementById('ts').textContent=new Date(j.ts).toLocaleTimeString();"
      "}catch(e){}}"
      "setInterval(poll,1000);poll();"
      "</script>"
      "</body></html>";
  server.send(200, "text/html", html);
}

static void print_self_check() {
  Serial.println("========== SELF CHECK ==========");
  Serial.printf("[CHK] Camera : %s\n", g_cameraReady ? "OK" : "FAIL");
  Serial.printf("[CHK] HX711  : %s (DT=%d SCK=%d)\n", g_hx711Ready ? "OK" : "FAIL", HX711_DT_PIN, HX711_SCK_PIN);
  Serial.printf("[CHK] Servo  : %s (PIN=%d)\n", g_servoReady ? "OK" : "FAIL", SERVO_PIN);
  Serial.printf("[CHK] Feed   : low<%.1fg target=%.1fg full<%.1fg timeout=%lums cooldown=%lums\n",
                LOW_FOOD_THRESHOLD, TARGET_FEED_WEIGHT, FULL_FOOD_THRESHOLD,
                (unsigned long)FEED_TIMEOUT_MS, (unsigned long)FEED_COOLDOWN_MS);
  Serial.printf("[CHK] WiFi   : %s\n", WiFi.status() == WL_CONNECTED ? "OK" : "FAIL");
  Serial.printf("[CHK] IP     : %s\n", WiFi.localIP().toString().c_str());
  Serial.println("================================");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n[BOOT] %s\n", FW_TAG);

  if (!init_camera()) {
    Serial.println("[FATAL] camera init failed");
    while (true) delay(1000);
  }
  init_hx711();
  init_servo();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("[WIFI] Connected. IP: %s\n", WiFi.localIP().toString().c_str());

  server.on("/", HTTP_GET, handle_root);
  server.on("/metrics", HTTP_GET, handle_metrics);
  server.on("/testServo", HTTP_GET, handle_test_servo);
  server.begin();
  Serial.println("[HTTP] Server started.");
  print_self_check();
}

void loop() {
  server.handleClient();
  uint32_t now = millis();
  if (now >= g_nextInferAt) {
    run_inference_once();
    g_nextInferAt = now + INFER_EVERY_MS;
  }
  if (now >= g_nextWeightAt) {
    update_weight_nonblocking();
    g_nextWeightAt = now + WEIGHT_READ_EVERY_MS;
  }
  if (now >= g_nextFeedStateAt) {
    run_feed_state_machine();
    g_nextFeedStateAt = now + FEED_STATE_TICK_MS;
  }
  if (g_servoTestActive && now >= g_servoCloseAt) {
    g_servo.write(servo_apply_angle(SERVO_CLOSED_ANGLE));
    g_servoTestActive = false;
  }
  delay(2);
}
