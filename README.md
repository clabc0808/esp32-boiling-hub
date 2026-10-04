# ESP32 Boiling Hub

Watch your hob with an ESP32-S3 camera. Get a phone notification when water boils, with a tap-to-view link to the exact video clip the AI analyzed.

## How it works

```
ESP32-S3 Camera → Bridge (Python) → Gemini Vision → MQTT → Home Assistant → Your phone
```

1. **ESP32-S3** watches the hob. Two modes:
   - **Production**: When Home Assistant says the hob is on (MQTT `kitchen/hob`), records 10s MJPEG video every 30s, POSTs to the bridge at `/upload`.
   - **DVR Burst**: Publish `ON` to MQTT `hob/cam_test` for a one-shot 10-frame still burst → bridge auto-stitches to MP4 → Gemini. Useful for testing without heating the hob.
2. **Bridge** (Python, runs on a local server like a Raspberry Pi or mini PC):
   - Receives video clips (`/upload`) or DVR bursts (`/dvr`)
   - Transcodes/stitches to H.264 MP4 via ffmpeg
   - Sends to Gemini Vision API for boiling detection
   - Publishes verdict `{"boiling": bool, "clip_url": "...", ...}` to MQTT `hob/boiling`
3. **Home Assistant** automation notifies your phone on every verdict. Tapping the notification opens the exact MP4 clip Gemini analyzed (served by the bridge at `/clip/<name>.mp4`).

## Hardware

### ESP32-S3 Camera Board
- ESP32-S3 with **8MB PSRAM** (e.g., ESP32-S3-CAM-Board-N16R8 — 16MB flash / 8MB octal PSRAM)
- OV5640 camera module (5MP)
- microSD card (for video/burst buffering)

**Arduino IDE settings:**
- Board: "ESP32S3 Dev Module"
- USB CDC On Boot: Enabled
- Flash Size: 16MB
- Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)"
- PSRAM: "OPI PSRAM" (mandatory for octal PSRAM)
- Upload Speed: 921600
- Libraries: PubSubClient (esp_camera, SD_MMC, WiFi, HTTPClient ship with the ESP32 Arduino core)

### Bridge Server
- Any Linux machine on your LAN (Raspberry Pi, mini PC, NAS, etc.)
- Python 3.8+, ffmpeg, access to an MQTT broker
- Tested on Ubuntu 24.04/26.04

## Setup

### 1. Bridge server

```bash
# Install dependencies
pip install paho-mqtt
# ffmpeg: apt install ffmpeg  (or use your distro's package)

# Configure
mkdir -p ~/hob-watch
cp bridge/config.env.example ~/hob-watch/config.env
# Edit ~/hob-watch/config.env — at minimum set GEMINI_API_KEY and HOB_BRIDGE_HOST

# Run
python3 bridge/bridge.py
```

For production, run as a systemd user service:
```bash
# See bridge/bridge.service.example (adapt paths, then):
systemctl --user enable --now hob-bridge
```

**Bridge endpoints:**

| Endpoint | Method | Purpose |
|---|---|---|
| `/upload` | POST | ESP32 posts MJPEG AVI clips (production video) |
| `/dvr` | POST | ESP32 posts DVR burst JPEGs (10 frames → auto-stitch → Gemini) |
| `/snapshot` | POST | Single JPEG still (testing / focus tuning) |
| `/clip/<name>.mp4` | GET | Serve a processed clip (notification tap-to-view links) |
| `/test` | GET | Live-updating page showing the latest snapshot |
| `/view` | GET | Play the latest processed clip in a browser |
| `/snapshot.jpg` | GET | Latest still image |

### 2. ESP32 firmware

1. Open `esp32/esp32-boiling-hub.ino` in Arduino IDE.
2. Fill in your credentials at the top of the file:
   ```cpp
   const char* WIFI_SSID     = "YOUR_WIFI_SSID";
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   const char* MQTT_HOST     = "YOUR_MQTT_BROKER_IP";  // e.g. Home Assistant Mosquitto
   const char* MQTT_USER     = "YOUR_MQTT_USER";
   const char* MQTT_PASS     = "YOUR_MQTT_PASSWORD";
   const char* UPLOAD_URL    = "http://YOUR_BRIDGE_HOST_IP:8099/upload";
   const char* DVR_URL       = "http://YOUR_BRIDGE_HOST_IP:8099/dvr";
   ```
3. Flash to your ESP32-S3 with the board settings above.

### 3. Home Assistant

**Publish hob state to MQTT** (so the ESP32 knows when to record):

```yaml
automation:
  - alias: "Hob watch - publish cooking state to MQTT"
    trigger:
      - platform: state
        entity_id: sensor.hob_operation_state
        to: "run"
      - platform: state
        entity_id: sensor.hob_operation_state
        not_to: "run"
    action:
      - service: mqtt.publish
        data:
          topic: kitchen/hob
          payload: "{{ 'on' if trigger.to_state.state == 'run' else 'off' }}"
          retain: true
```

> Adjust `sensor.hob_operation_state` to your hob's entity. Any sensor works — the ESP32 just needs `on`/`off` on `kitchen/hob`.

**Verdict → phone notification** (with tap-to-view the analyzed clip):

```yaml
automation:
  - alias: "Hob watch - notify boiling verdict"
    trigger:
      - platform: mqtt
        topic: hob/boiling
    action:
      - service: notify.notify
        data:
          title: "Hob Watch"
          message: "Boiling: {{ trigger.payload_json.boiling }}. Tap to view."
          data:
            clickAction: "{{ trigger.payload_json.clip_url }}"
```

> `notify.notify` targets all registered mobile-app devices. Replace with `notify.mobile_app_<your_phone>` to target specific devices.

## Project structure

```
esp32-boiling-hub/
├── esp32/
│   └── esp32-boiling-hub.ino   # ESP32-S3 firmware (production video + DVR burst)
├── bridge/
│   ├── bridge.py               # Python bridge: HTTP → ffmpeg → Gemini → MQTT
│   └── config.env.example      # Configuration template
└── README.md
```

## Key design decisions

### Why the OV5640 uses software JPEG
The OV5640's hardware JPEG path is dead on these ESP32-S3 boards (the sensor never emits JPEG data). The firmware captures **RGB565** and software-encodes each frame via `fmt2jpg()`. At XGA this takes ~670ms/frame — fine for 10s clips and bursts, but not for high-framerate video.

### Why XGA (1024×768), not higher
- XGA RGB565 = 1.5MB/frame. Two framebuffers = 3MB, leaving ~5MB of the 8MB PSRAM for the system, MQTT, HTTP, and encode scratch.
- SXGA (1280×1024) needs 2.6MB/frame × 3 buffers = 7.8MB — it works for bursts but leaves almost nothing for anything else, and software encode jumps to ~1.1s/frame.
- UXGA (1600×1200) doesn't fit at all: 4 buffers exceed 8MB, and 2 buffers starve the DMA.

### Why the camera is never deinitialized
`esp_camera_deinit()` followed by `esp_camera_init()` fails on the second run: the heap fragments (many small allocations from WiFi/MQTT/HTTP land between the freed framebuffer blocks) and the driver can't find contiguous 1.5MB blocks, even though total free PSRAM looks healthy. The fix: **allocate the framebuffers once at boot and never free them**. The DVR "borrows" a checked-out framebuffer as its encode scratch buffer instead of doing a separate 1.5MB `ps_malloc`. Zeroing RAM doesn't help — it's allocator layout, not stale data.

### DVR burst architecture (producer-consumer)
Phase 1 (capture) runs a FreeRTOS producer-consumer: the camera fills framebuffers while a writer task on Core 0 drains them to SD. With 2 buffers, the first 2 frames capture back-to-back (~190ms), then capture throttles to SD write speed (~580ms/frame at XGA). Total: 10 frames in ~6s.

Phase 2 reads each raw frame back from SD into a borrowed camera buffer and software-encodes to JPEG. Phase 3 POSTs the JPEGs to the bridge.

### SD card is the bottleneck
Measured ~2.3–2.7 MB/s on 1-bit SDMMC. A 1.5MB XGA raw frame takes ~580ms to write. This is what limits burst rate, not the sensor.

## Troubleshooting

- **Camera init fails**: check the pin definitions at the top of the `.ino` match your board. The defaults suit common ESP32-S3-N16R8 40-pin camera boards (GOOUUU/Freenove style).
- **`fb_get()` timeouts**: you don't have enough free PSRAM for the configured `fb_count` — reduce resolution or `fb_count`.
- **Bridge 400 on `/upload`**: it only accepts MJPEG AVI (`RIFF....AVI ` magic). `/dvr` and `/snapshot` only accept JPEG (`FF D8` magic).
- **No verdict**: check `GEMINI_API_KEY` in `config.env`, and that `HOB_DRY_RUN` is `0`. Watch bridge logs with `journalctl --user -u hob-bridge -f`.
- **Notification tap does nothing**: `HOB_BRIDGE_HOST` must be an IP/hostname your phone can reach on your LAN. `127.0.0.1` won't work from the phone.

## License

MIT — do what you want, no warranty.
