/*
 * esp32-boiling-hub.ino — ESP32-S3 hob boiling-water watcher
 *
 * PRODUCTION (kitchen/hob):
 *   - "on"  -> every 30s, record 10s MJPEG AVI to SD, POST to /upload
 *   - "off" -> idle
 *   Bridge transcodes, asks Gemini, publishes verdict to hob/boiling.
 *
 * DVR BURST (hob/cam_test):
 *   - "ON" -> one-shot: 10 XGA frames -> SD -> encode -> POST to /dvr
 *   - Bridge stitches to MP4, asks Gemini, publishes verdict.
 *
 * Camera: XGA 1024x768 RGB565, fb_count=2 (unified, no reinit between modes).
 *   2 x 1.5MB = 3MB. Phase 2 borrows a camera buffer as encode scratch.
 *   Camera stays initialized permanently (avoids deinit/reinit fragmentation).
 *
 * BOARD: ESP32S3 Dev Module, USB CDC On Boot Enabled, 16MB Flash,
 *   16M (3MB APP/9.9MB FATFS), OPI PSRAM, 921600 upload.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "SD_MMC.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* ----------------------------- CONFIGURATION ----------------------------- */
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MQTT_HOST  = "YOUR_MQTT_BROKER_IP";
const int   MQTT_PORT  = 1883;
const char* MQTT_TOPIC = "kitchen/hob";       // "on"/"off" from HA
const char* MQTT_TEST_TOPIC = "hob/cam_test"; // "ON" = DVR burst (one-shot)
const char* MQTT_USER  = "YOUR_MQTT_USER";
const char* MQTT_PASS  = "YOUR_MQTT_PASSWORD";

const char* UPLOAD_URL = "http://YOUR_BRIDGE_HOST_IP:8099/upload";
const char* DVR_URL    = "http://YOUR_BRIDGE_HOST_IP:8099/dvr";

const unsigned long CLIP_MS   = 10000;  // record 10s ...
const unsigned long PERIOD_MS = 30000;  // ... every 30s while hob is on

// DVR config
#define DVR_W 1024
#define DVR_H 768
#define DVR_FRAME_BYTES (DVR_W * DVR_H * 2)  // 1,572,864
#define DVR_N_SHOTS 10
#define DVR_JPEG_QUALITY 30
#define DVR_FB_DEPTH 2

// Pins
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13
#define SD_CLK 39
#define SD_CMD 38
#define SD_D0  40
#define LED_GPIO 2

#define CAM_W 1024
#define CAM_H 768

/* --------------------------------- STATE --------------------------------- */
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
volatile bool hobActive = false;
volatile bool dvrRequested = false;
volatile bool dvrRunning = false;

// DVR FreeRTOS
static QueueHandle_t dvrQueue = NULL;
static SemaphoreHandle_t dvrDoneSem = NULL;
static volatile int dvrFilesWritten = 0;
static volatile bool dvrAbort = false;
static size_t dvrFrameBytes = 0;

static unsigned long lastWifiAttempt = 0;

void led(bool on) {
  if (LED_GPIO >= 0) digitalWrite(LED_GPIO, on ? HIGH : LOW);
}

/* ------------------------------ WiFi / MQTT ------------------------------ */
void wifiEnsure() {
  if (WiFi.status() == WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lastWifiAttempt < 30000) return;
  lastWifiAttempt = now;
  Serial.printf("[wifi] reconnecting to %s...", WIFI_SSID);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500); Serial.print("."); tries++;
    if (WiFi.status() == WL_CONNECT_FAILED) break;
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " ok" : " FAILED");
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String t(topic), p;
  for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
  p.trim();
  Serial.printf("[mqtt] %s = %s\n", t.c_str(), p.c_str());
  if (t == MQTT_TOPIC) {
    hobActive = (p == "on");
    Serial.printf("[hob] active=%d\n", hobActive);
  } else if (t == MQTT_TEST_TOPIC && p == "ON" && !dvrRunning && !dvrRequested) {
    Serial.println("[dvr] trigger received");
    dvrRequested = true;
  }
}

void mqttEnsure() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  Serial.print("[mqtt] connecting... ");
  String cid = "hob-merged-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASS);
  Serial.println(ok ? "ok" : "FAILED");
  if (ok) {
    mqtt.subscribe(MQTT_TOPIC);
    mqtt.subscribe(MQTT_TEST_TOPIC);
  }
}

/* -------------------------------- Camera --------------------------------- */
bool initCamera() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_XGA;
  c.jpeg_quality = 12;
  c.fb_count = DVR_FB_DEPTH;  // 2 x 1.5MB
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[cam] init FAILED 0x%x\n", err);
    return false;
  }
  Serial.println("[cam] init ok, XGA, fb_count=2");
  return true;
}

/* --------------------------- MINIMAL AVI WRITER -------------------------- */
// Writes a playable MJPEG AVI. All integers little-endian (native on ESP32).

static void w32(File &f, uint32_t v) { f.write((uint8_t*)&v, 4); }

struct AviWriter {
  File f;
  uint32_t riffSizePos, hdrlLenPos, moviLenPos;
  uint32_t avihFramesPos, strhRatePos, strhLenPos, strlLenPos;
  uint32_t moviDataStart;
  uint32_t frameCount = 0;
  // index entries kept small: 10 s @ ~20 fps worst case = 200 entries
  struct Idx { uint32_t offset, size; };
  Idx idx[400];
  uint16_t idxCount = 0;

  bool open(const char* path, uint16_t w, uint16_t h) {
    f = SD_MMC.open(path, FILE_WRITE);
    if (!f) return false;

    f.write((uint8_t*)"RIFF", 4);
    riffSizePos = f.position(); w32(f, 0);
    f.write((uint8_t*)"AVI ", 4);

    f.write((uint8_t*)"LIST", 4);
    hdrlLenPos = f.position(); w32(f, 0);
    f.write((uint8_t*)"hdrl", 4);

    // avih
    f.write((uint8_t*)"avih", 4); w32(f, 56);
    w32(f, 66667);            // dwMicroSecPerFrame (15 fps nominal, fixed up later)
    w32(f, 0);                // dwMaxBytesPerSec
    w32(f, 0);                // dwPaddingGranularity
    w32(f, 0x10);             // dwFlags = AVIF_HASINDEX
    avihFramesPos = f.position(); w32(f, 0);  // dwTotalFrames (fixup)
    w32(f, 0);                // dwInitialFrames
    w32(f, 1);                // dwStreams
    w32(f, 0);                // dwSuggestedBufferSize
    w32(f, w); w32(f, h);
    for (int i = 0; i < 4; i++) w32(f, 0);    // dwReserved[4]

    // strl
    f.write((uint8_t*)"LIST", 4);
    strlLenPos = f.position(); w32(f, 0);
    f.write((uint8_t*)"strl", 4);

    // strh (AVISTREAMHEADER = 64 bytes)
    f.write((uint8_t*)"strh", 4); w32(f, 64);
    f.write((uint8_t*)"vids", 4);             // fccType
    f.write((uint8_t*)"MJPG", 4);             // fccHandler
    w32(f, 0); w32(f, 0);                     // dwFlags; wPriority+wLanguage
    w32(f, 0);                                // dwInitialFrames
    w32(f, 1);                                // dwScale
    strhRatePos = f.position(); w32(f, 15);   // dwRate (fixup)
    w32(f, 0);                                // dwStart
    strhLenPos = f.position(); w32(f, 0);     // dwLength (fixup)
    w32(f, 0);                                // dwSuggestedBufferSize
    w32(f, 0xFFFFFFFF);                       // dwQuality
    w32(f, 0);                                // dwSampleSize
    w32(f, 0); w32(f, 0); w32(f, w); w32(f, h); // rcFrame

    // strf (BITMAPINFOHEADER)
    f.write((uint8_t*)"strf", 4); w32(f, 40);
    w32(f, 40); w32(f, w); w32(f, h);
    f.write((uint8_t*)"\x01\x00\x18\x00", 4); // planes=1, bitcount=24
    f.write((uint8_t*)"MJPG", 4);             // biCompression
    w32(f, (uint32_t)w * h * 3);              // biSizeImage (estimate)
    w32(f, 0); w32(f, 0); w32(f, 0); w32(f, 0);

    uint32_t strlEnd = f.position();
    f.seek(strlLenPos); w32(f, strlEnd - strlLenPos - 4); f.seek(strlEnd);

    // movi
    f.write((uint8_t*)"LIST", 4);
    moviLenPos = f.position(); w32(f, 0);
    f.write((uint8_t*)"movi", 4);
    moviDataStart = f.position();
    return true;
  }

  void addFrame(const uint8_t* data, uint32_t len) {
    if (idxCount >= 400) return;
    f.write((uint8_t*)"00dc", 4);
    w32(f, len);
    uint32_t dataPos = f.position();
    f.write(data, len);
    if (len & 1) f.write((uint8_t)0);  // pad to even
    idx[idxCount].offset = dataPos - moviDataStart;
    idx[idxCount].size = len;
    idxCount++;
    frameCount++;
  }

  // fps_x100 = frames per second * 100, for exact back-patch
  void close(uint32_t fps_x100, uint16_t w, uint16_t h) {
    uint32_t moviEnd = f.position();
    f.seek(moviLenPos); w32(f, moviEnd - moviLenPos - 4); f.seek(moviEnd);

    f.write((uint8_t*)"idx1", 4);
    w32(f, idxCount * 16);
    for (uint16_t i = 0; i < idxCount; i++) {
      f.write((uint8_t*)"00dc", 4);
      w32(f, 0x10);
      w32(f, idx[i].offset);
      w32(f, idx[i].size);
    }

    uint32_t end = f.position();
    f.seek(riffSizePos); w32(f, end - 8);
    f.seek(hdrlLenPos);  w32(f, moviLenPos - hdrlLenPos - 4);
    f.seek(avihFramesPos); w32(f, frameCount);
    uint32_t fps = fps_x100 / 100;
    if (fps < 1) fps = 1;
    f.seek(strhRatePos); w32(f, fps);
    f.seek(strhLenPos);  w32(f, frameCount);
    // fix avih microsec/frame to match measured fps
    f.seek(avihFramesPos - 16); w32(f, 1000000 / fps);
    f.close();
  }
};

/* ------------------------------ RECORDING -------------------------------- */

bool recordClip(const char* path, unsigned long ms) {
  AviWriter avi;
  if (!avi.open(path, CAM_W, CAM_H)) {
    Serial.println("[rec] SD open failed");
    return false;
  }
  Serial.println("[rec] recording...");
  led(true);
  unsigned long t0 = micros();
  uint32_t frames = 0;
  while (micros() - t0 < ms * 1000UL) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;
    // software JPEG encode (see OV5640 quirk above)
    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    if (fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 12, &jpg, &jpg_len)) {
      avi.addFrame(jpg, jpg_len);
      free(jpg);
      frames++;
    }
    esp_camera_fb_return(fb);
    if ((frames % 20) == 0) delay(1);  // feed the task watchdog
  }
  unsigned long elapsed = micros() - t0;
  uint32_t fps_x100 = elapsed ? (uint32_t)((uint64_t)frames * 1000000 * 100 / elapsed) : 1500;
  avi.close(fps_x100, CAM_W, CAM_H);
  led(false);
  Serial.printf("[rec] done: %u frames in %.1fs (%.1f fps)\n",
                frames, elapsed / 1e6, frames * 1e6 / elapsed);
  return true;
}

bool uploadClip(const char* path) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) { Serial.println("[up] open failed"); return false; }
  size_t len = f.size();
  Serial.printf("[up] POST %u bytes -> %s\n", (unsigned)len, UPLOAD_URL);

  HTTPClient http;
  http.setTimeout(90000);
  http.begin(UPLOAD_URL);
  http.addHeader("Content-Type", "video/x-msvideo");
  int code = http.sendRequest("POST", &f, len);
  f.close();
  Serial.printf("[up] response: %d %s\n", code, http.getString().c_str());
  http.end();
  return code == 200;
}

/* ------------------------------ DVR (burst) ------------------------------- */
void dvrWriterTask(void* param) {
  (void)param;
  int idx = 0;
  while (idx < DVR_N_SHOTS && !dvrAbort) {
    camera_fb_t* fb = NULL;
    if (xQueueReceive(dvrQueue, &fb, pdMS_TO_TICKS(2000)) != pdTRUE) continue;
    if (!fb) continue;
    char path[32];
    snprintf(path, sizeof(path), "/burst/raw%02d.bin", idx);
    File f = SD_MMC.open(path, FILE_WRITE);
    bool ok = f && (f.write(fb->buf, fb->len) == fb->len);
    if (f) f.close();
    esp_camera_fb_return(fb);
    if (ok) { idx++; dvrFilesWritten = idx; }
    else break;
  }
  xSemaphoreGive(dvrDoneSem);
  vTaskDelete(NULL);
}

bool dvrCapture() {
  Serial.println("[dvr] PHASE 1: CAPTURE");
  dvrQueue = xQueueCreate(DVR_FB_DEPTH, sizeof(camera_fb_t*));
  dvrDoneSem = xSemaphoreCreateBinary();
  if (!dvrQueue || !dvrDoneSem) return false;
  dvrFilesWritten = 0; dvrAbort = false; dvrFrameBytes = 0;

  if (xTaskCreatePinnedToCore(dvrWriterTask, "dvrW", 8192, NULL, 1, NULL, 0) != pdPASS)
    return false;

  bool ok = true;
  int captured = 0;
  while (captured < DVR_N_SHOTS && ok) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { delay(200); continue; }
    if (dvrFrameBytes == 0) {
      dvrFrameBytes = fb->len;
      Serial.printf("[dvr] frame size: %u\n", (unsigned)dvrFrameBytes);
    }
    if (xQueueSend(dvrQueue, &fb, pdMS_TO_TICKS(5000)) != pdTRUE) {
      esp_camera_fb_return(fb); ok = false; break;
    }
    captured++;
    Serial.printf("[dvr] captured %d/%d\n", captured, DVR_N_SHOTS);
  }

  if (ok) {
    if (xSemaphoreTake(dvrDoneSem, pdMS_TO_TICKS(90000)) != pdTRUE) ok = false;
  } else {
    dvrAbort = true;
    xSemaphoreTake(dvrDoneSem, pdMS_TO_TICKS(10000));
  }
  // drain
  if (dvrQueue) {
    camera_fb_t* fb;
    while (xQueueReceive(dvrQueue, &fb, 0) == pdTRUE)
      if (fb) esp_camera_fb_return(fb);
    vQueueDelete(dvrQueue);
  }
  vSemaphoreDelete(dvrDoneSem);
  dvrQueue = NULL;
  Serial.printf("[dvr] PHASE 1 %s: %d/%d\n", ok?"OK":"FAIL", dvrFilesWritten, DVR_N_SHOTS);
  return ok && dvrFilesWritten == DVR_N_SHOTS;
}

bool dvrEncode() {
  Serial.println("[dvr] PHASE 2: ENCODE");
  if (!dvrFrameBytes) return false;
  // Borrow a camera buffer as scratch
  camera_fb_t* scratch = esp_camera_fb_get();
  if (!scratch || scratch->len < dvrFrameBytes) {
    if (scratch) esp_camera_fb_return(scratch);
    return false;
  }
  int done = 0;
  for (int i = 0; i < DVR_N_SHOTS; i++) {
    char rp[32], jp[32];
    snprintf(rp, sizeof(rp), "/burst/raw%02d.bin", i);
    snprintf(jp, sizeof(jp), "/burst/jpg%02d.jpg", i);
    File fr = SD_MMC.open(rp, FILE_READ);
    if (!fr) continue;
    if (fr.read(scratch->buf, dvrFrameBytes) != dvrFrameBytes) { fr.close(); continue; }
    fr.close();
    uint8_t* jpg = NULL; size_t jl = 0;
    if (!fmt2jpg(scratch->buf, dvrFrameBytes, DVR_W, DVR_H, PIXFORMAT_RGB565,
                 DVR_JPEG_QUALITY, &jpg, &jl) || !jpg) continue;
    File fj = SD_MMC.open(jp, FILE_WRITE);
    bool w = fj && (fj.write(jpg, jl) == jl);
    if (fj) fj.close();
    free(jpg);
    if (w) { done++; Serial.printf("[dvr] enc %d: %u bytes\n", i, (unsigned)jl); }
  }
  esp_camera_fb_return(scratch);
  Serial.printf("[dvr] PHASE 2: %d/%d\n", done, DVR_N_SHOTS);
  return done == DVR_N_SHOTS;
}

bool dvrStream() {
  Serial.println("[dvr] PHASE 3: STREAM");
  wifiEnsure();
  if (WiFi.status() != WL_CONNECTED) return false;
  int done = 0;
  for (int i = 0; i < DVR_N_SHOTS; i++) {
    char jp[32]; snprintf(jp, sizeof(jp), "/burst/jpg%02d.jpg", i);
    File f = SD_MMC.open(jp, FILE_READ);
    if (!f) continue;
    size_t len = f.size();
    uint8_t* buf = (uint8_t*)ps_malloc(len);
    if (!buf) { f.close(); continue; }
    f.read(buf, len); f.close();
    HTTPClient http;
    http.begin(DVR_URL);
    http.addHeader("Content-Type", "image/jpeg");
    http.setTimeout(15000);
    int code = http.POST(buf, len);
    http.end(); free(buf);
    Serial.printf("[dvr] stream %d -> %d\n", i, code);
    if (code == 200) done++;
    delay(150);
  }
  Serial.printf("[dvr] PHASE 3: %d/%d\n", done, DVR_N_SHOTS);
  return done == DVR_N_SHOTS;
}

void runDVR() {
  dvrRunning = true; led(true);
  Serial.println("\n[dvr] ===== START =====");
  bool c = dvrCapture();
  bool e = c ? dvrEncode() : false;
  bool s = e ? dvrStream() : false;
  Serial.printf("[dvr] ===== DONE: cap=%s enc=%s stream=%s =====\n",
                c?"OK":"FAIL", e?"OK":"FAIL", s?"OK":"FAIL");
  led(false); dvrRunning = false;
}

/* --------------------------------- Setup ---------------------------------- */
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n[hob-merged] boot");
  pinMode(LED_GPIO, OUTPUT); led(false);

  if (!psramFound()) Serial.println("[warn] no PSRAM");
  Serial.printf("[psram] %u\n", ESP.getFreePsram());

  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[sd] FAILED, halting"); while(1) delay(1000);
  }
  SD_MMC.mkdir("/burst");
  Serial.println("[sd] ok");

  // Camera stays initialized permanently (no deinit/fragmentation)
  if (!initCamera()) {
    Serial.println("[FATAL] cam"); while(1) delay(1000);
  }

  wifiEnsure();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqttEnsure();
  Serial.println("[hob-merged] ready. kitchen/hob=video, hob/cam_test=DVR burst");
}

/* ---------------------------------- Loop ---------------------------------- */
void loop() {
  wifiEnsure();
  mqttEnsure();
  mqtt.loop();

  // DVR has priority (one-shot)
  if (dvrRequested && !dvrRunning) {
    dvrRequested = false;
    runDVR();
    return;
  }

  // Production: hob on -> video every 30s
  if (!hobActive) { delay(200); return; }

  unsigned long cycleStart = millis();
  const char* clipPath = "/clip.avi";
  if (recordClip(clipPath, CLIP_MS)) {
    if (!uploadClip(clipPath)) {
      Serial.println("[up] upload failed, will retry next cycle");
    }
    SD_MMC.remove(clipPath);
  }

  while (millis() - cycleStart < PERIOD_MS) {
    mqtt.loop();
    if (!hobActive || dvrRequested) break;
    delay(150);
  }
}
