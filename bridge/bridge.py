#!/usr/bin/env python3
"""
esp32-boiling-hub bridge: ESP32-S3 hob camera -> Gemini boiling verdict -> MQTT.

Pipeline per clip:
  1. S3 POSTs MJPEG AVI to http://ppl01:8099/upload
  2. ffmpeg transcodes AVI -> H.264 MP4
  3. MP4 sent to Gemini; prompt demands strict JSON {"boiling": bool}
  4. Verdict published (retained) to MQTT topic hob/boiling for Home Assistant

Config via environment or ~/hob-watch/config.env (KEY=VALUE lines):
  GEMINI_API_KEY   (required for live mode; empty in dry-run)
  GEMINI_MODEL     default: gemini-2.5-flash
  HOB_HTTP_PORT    default: 8099
  HOB_BRIDGE_HOST  default: 127.0.0.1 (public IP for clip_url in verdicts)
  HOB_MQTT_HOST    default: 127.0.0.1
  HOB_MQTT_PORT    default: 1883
  HOB_TOPIC        default: hob/boiling
  HOB_FFMPEG       default: /home/ppl/bin/ffmpeg
  HOB_EVERY_N      default: 1 (send every Nth clip to Gemini)
  HOB_DRY_RUN      default: 0 (1 = skip Gemini, publish dry-run verdict)
  HOB_KEEP_DONE    default: 50 (trim processed clips beyond this many)
"""

import base64
import json
import logging
import os
import queue
import re
import shutil
import subprocess
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

import paho.mqtt.client as mqtt

HOME = Path.home()
BASE = HOME / "hob-watch"
INBOX = BASE / "inbox"
DONE = BASE / "done"
SNAP = BASE / "snapshot.jpg"
BURSTS = BASE / "bursts"
BURSTS.mkdir(exist_ok=True)
BURST_KEEP = 40
DVRDIR = BASE / "dvr"
DVRDIR.mkdir(exist_ok=True)
DVR_FRAMES = 10
_dvr_lock = threading.Lock()

_cfg = BASE / "config.env"
if _cfg.exists():
    for _line in _cfg.read_text().splitlines():
        _line = _line.strip()
        if _line and not _line.startswith("#") and "=" in _line:
            _k, _v = _line.split("=", 1)
            os.environ.setdefault(_k.strip(), _v.strip().strip('"').strip("'"))


def env(name, default):
    return os.environ.get(name, default)


HTTP_PORT = int(env("HOB_HTTP_PORT", "8099"))
BRIDGE_HOST = env("HOB_BRIDGE_HOST", "127.0.0.1")  # public IP/hostname for clip URLs
MQTT_HOST = env("HOB_MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(env("HOB_MQTT_PORT", "1883"))
MQTT_USER = env("HOB_MQTT_USER", "")
MQTT_PASS = env("HOB_MQTT_PASS", "")
TOPIC = env("HOB_TOPIC", "hob/boiling")
FFMPEG = env("HOB_FFMPEG", str(HOME / "bin" / "ffmpeg"))
GEMINI_KEY = env("GEMINI_API_KEY", "")
GEMINI_MODEL = env("GEMINI_MODEL", "gemini-2.5-flash")
EVERY_N = max(1, int(env("HOB_EVERY_N", "1")))
DRY_RUN = env("HOB_DRY_RUN", "0") == "1"
KEEP_DONE = int(env("HOB_KEEP_DONE", "50"))

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
)
log = logging.getLogger("hob-watch")

work = queue.Queue()
clip_counter = 0
counter_lock = threading.Lock()

PROMPT = (
    "You are watching a pot on an induction hob. Look at the water surface. "
    "Is the water at a rolling boil (vigorous bubbling across the surface)? "
    'Reply with ONLY this JSON and nothing else: {"boiling": true} or {"boiling": false}'
)


def transcode(src: Path, dst: Path):
    cmd = [
        FFMPEG, "-y", "-v", "error", "-i", str(src),
        "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
        "-pix_fmt", "yuv420p", "-an", str(dst),
    ]
    subprocess.run(cmd, check=True, timeout=120)


def ask_gemini(mp4_path: Path) -> bool:
    data = base64.b64encode(mp4_path.read_bytes()).decode()
    body = {
        "contents": [{"parts": [
            {"text": PROMPT},
            {"inline_data": {"mime_type": "video/mp4", "data": data}},
        ]}],
        "generationConfig": {
            "temperature": 0,
            "maxOutputTokens": 32,
            "responseMimeType": "application/json",
        },
    }
    req = Request(
        f"https://generativelanguage.googleapis.com/v1beta/models/"
        f"{GEMINI_MODEL}:generateContent",
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json", "x-goog-api-key": GEMINI_KEY},
    )
    try:
        with urlopen(req, timeout=120) as resp:
            payload = json.loads(resp.read())
    except HTTPError as e:
        detail = e.read().decode()[:500]
        raise RuntimeError(f"Gemini HTTP {e.code}: {detail}")
    except URLError as e:
        raise RuntimeError(f"Gemini network error: {e}")
    try:
        text = payload["candidates"][0]["content"]["parts"][0]["text"]
    except (KeyError, IndexError):
        raise RuntimeError(f"Unexpected Gemini response: {str(payload)[:300]}")
    m = re.search(r"\{[^{}]*\}", text)
    if not m:
        raise RuntimeError(f"No JSON in Gemini reply: {text[:200]}")
    val = json.loads(m.group(0)).get("boiling")
    if not isinstance(val, bool):
        raise RuntimeError(f"Bad verdict value: {text[:200]}")
    return val


_mqtt_client = None
_mqtt_lock = threading.Lock()


def get_mqtt():
    global _mqtt_client
    with _mqtt_lock:
        if _mqtt_client is None:
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            if MQTT_USER:
                c.username_pw_set(MQTT_USER, MQTT_PASS)
            c.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
            c.loop_start()
            _mqtt_client = c
        return _mqtt_client


def publish_verdict(verdict: dict):
    get_mqtt().publish(TOPIC, json.dumps(verdict), qos=1, retain=True)
    log.info("published %s -> %s", TOPIC, verdict)



def process_dvr_run():
    """Stitch oldest DVR_FRAMES JPEGs to MP4, ask Gemini, publish verdict."""
    global clip_counter
    with _dvr_lock:
        files = sorted(DVRDIR.glob("dvr_*.jpg"))
        if len(files) < DVR_FRAMES:
            return
        batch = files[:DVR_FRAMES]
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    lst = BASE / ("dvr_%s.txt" % ts)
    mp4 = BASE / ("dvr_%s.mp4" % ts)
    try:
        with open(lst, "w") as f:
            for fp in batch:
                f.write("file '%s'\n" % fp)
        r = subprocess.run(
            [FFMPEG, "-y", "-f", "concat", "-safe", "0", "-i", str(lst),
             "-framerate", "2", "-c:v", "libx264", "-pix_fmt", "yuv420p",
             str(mp4)],
            capture_output=True, timeout=60)
        if r.returncode != 0 or not mp4.exists():
            log.error("dvr ffmpeg failed: %s", r.stderr.decode()[-300:])
            return
        log.info("dvr stitched %d frames -> %s (%d bytes)",
                 len(batch), mp4.name, mp4.stat().st_size)
        with counter_lock:
            clip_counter += 1
            n = clip_counter
        if n % EVERY_N != 0:
            log.info("dvr: skipped by HOB_EVERY_N=%d", EVERY_N)
            verdict = None
        elif DRY_RUN:
            log.info("dvr: dry run, skipping Gemini")
            verdict = {"boiling": False, "dry_run": True}
        else:
            if not GEMINI_KEY:
                raise RuntimeError("GEMINI_API_KEY not set")
            t0 = time.time()
            boiling = ask_gemini(mp4)
            log.info("dvr: gemini verdict=%s in %.1fs", boiling, time.time() - t0)
            verdict = {"boiling": boiling}
        if verdict is not None:
            verdict.update({
                "ts": datetime.now(timezone.utc).isoformat(),
                "clip": mp4.name,
                "clip_url": "http://%s:%d/clip/" % (BRIDGE_HOST, HTTP_PORT) + mp4.name,
                "model": GEMINI_MODEL,
                "source": "dvr",
            })
            publish_verdict(verdict)
    except Exception as e:
        log.error("dvr process failed: %s", e)
    finally:
        for fp in batch:
            fp.unlink(missing_ok=True)
        lst.unlink(missing_ok=True)
        if mp4.exists():
            (DONE / mp4.name).write_bytes(mp4.read_bytes())
            mp4.unlink(missing_ok=True)

def trim_done():
    stems = sorted({p.stem for p in DONE.glob("hob_*")})
    for stem in stems[: max(0, len(stems) - KEEP_DONE)]:
        for p in DONE.glob(stem + ".*"):
            p.unlink(missing_ok=True)


def process_clip(path: Path):
    global clip_counter
    with counter_lock:
        clip_counter += 1
        n = clip_counter
    stem = path.stem
    mp4 = INBOX / (stem + ".mp4")
    try:
        if n % EVERY_N != 0:
            log.info("%s: skipped by HOB_EVERY_N=%d", path.name, EVERY_N)
            verdict = None
        elif DRY_RUN:
            log.info("%s: dry run, skipping Gemini", path.name)
            verdict = {"boiling": False, "dry_run": True}
        else:
            if not GEMINI_KEY:
                raise RuntimeError("GEMINI_API_KEY not set")
            t0 = time.time()
            transcode(path, mp4)
            log.info("%s: transcoded in %.1fs", path.name, time.time() - t0)
            t0 = time.time()
            boiling = ask_gemini(mp4)
            log.info("%s: gemini verdict=%s in %.1fs",
                     path.name, boiling, time.time() - t0)
            verdict = {"boiling": boiling}
        if verdict is not None:
            verdict.update({
                "ts": datetime.now(timezone.utc).isoformat(),
                "clip": path.name,
                "clip_url": "http://%s:%d/clip/" % (BRIDGE_HOST, HTTP_PORT) + mp4.name,
                "model": GEMINI_MODEL,
            })
            publish_verdict(verdict)
    except Exception as e:
        log.error("%s: FAILED: %s", path.name, e)
    finally:
        try:
            shutil.move(str(path), DONE / path.name)
            if mp4.exists():
                shutil.move(str(mp4), DONE / mp4.name)
        except Exception as e:
            log.error("move to done failed: %s", e)
        trim_done()


def worker():
    while True:
        process_clip(work.get())


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]  # ignore query strings (e.g. cache-busters)
        if path == "/health":
            self._json(200, {"ok": True})
        elif path == "/latest":
            mp4s = sorted(DONE.glob("hob_*.mp4"))
            if not mp4s:
                self.send_response(404)
                self.end_headers()
                return
            data = mp4s[-1].read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "video/mp4")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        elif path == "/view":
            html = ("<html><head><meta charset='utf-8'><title>hob cam</title></head><body style='margin:0;background:#111'>"
                    "<video src='/latest' style='width:100vw;height:100vh;object-fit:contain' autoplay muted loop controls></video>"
                    "<script>setTimeout(()=>location.reload(),35000)</script>"
                    "</body></html>").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        elif path.startswith("/clip/"):
            name = path[len("/clip/"):]
            if not name or "/" in name or ".." in name or not name.endswith(".mp4"):
                self.send_response(400); self.end_headers(); return
            fp = DONE / name
            if not fp.exists():
                self.send_response(404); self.end_headers(); return
            data = fp.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "video/mp4")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        elif path == "/snapshot.jpg":
            if not SNAP.exists():
                self.send_response(404)
                self.end_headers()
                return
            data = SNAP.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        elif path == "/test":
            html = (
                "<html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>hob cam test</title></head>"
                "<body style='margin:0;background:#111;color:#eee;font-family:sans-serif'>"
                "<div style='padding:10px'>TEST MODE &mdash; live stills (~1/s). "
                "Adjust focus/position and watch here.</div>"
                "<img id='img' src='/snapshot.jpg' "
                "style='width:100vw;max-height:88vh;object-fit:contain;background:#000'>"
                "<script>setInterval(()=>{"
                'document.getElementById("img").src='
                "'/snapshot.jpg?t='+Date.now()},400)"
                "</script></body></html>"
            ).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == "/dvr":
            length = int(self.headers.get("Content-Length", 0))
            if length <= 0 or length > 5 * 1024 * 1024:
                self.send_response(400)
                self.end_headers()
                return
            data = self.rfile.read(length)
            if not (data[:2] == b"\xff\xd8"):
                log.warning("rejected non-JPEG dvr frame (%d bytes)", len(data))
                self.send_response(400)
                self.end_headers()
                return
            (DVRDIR / ("dvr_%d.jpg" % int(time.time() * 1000))).write_bytes(data)
            SNAP.write_bytes(data)
            self._json(200, {"ok": True})
            threading.Thread(target=process_dvr_run, daemon=True).start()
            return
        if self.path == "/snapshot":
            length = int(self.headers.get("Content-Length", 0))
            if length <= 0 or length > 5 * 1024 * 1024:
                self.send_response(400)
                self.end_headers()
                return
            data = self.rfile.read(length)
            if not (data[:2] == b"\xff\xd8"):
                log.warning("rejected non-JPEG snapshot (%d bytes)", len(data))
                self.send_response(400)
                self.end_headers()
                return
            SNAP.write_bytes(data)
            # archive burst frames (keeps last BURST_KEEP, /test still shows latest)
            (BURSTS / ("snap_%d.jpg" % int(time.time() * 1000))).write_bytes(data)
            for old in sorted(BURSTS.glob("snap_*.jpg"))[:-BURST_KEEP]:
                old.unlink(missing_ok=True)
            self._json(200, {"ok": True})
            return
        if self.path != "/upload":
            self.send_response(404)
            self.end_headers()
            return
        length = int(self.headers.get("Content-Length", 0))
        if length <= 0 or length > 100 * 1024 * 1024:
            self.send_response(400)
            self.end_headers()
            return
        data = self.rfile.read(length)
        if not (data[:4] == b"RIFF" and data[8:12] == b"AVI "):
            log.warning("rejected non-AVI upload (%d bytes)", len(data))
            self.send_response(400)
            self.end_headers()
            return
        name = "hob_%s.avi" % datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
        (INBOX / name).write_bytes(data)
        log.info("received %s (%d bytes)", name, len(data))
        work.put(INBOX / name)
        self._json(200, {"ok": True})


def main():
    INBOX.mkdir(parents=True, exist_ok=True)
    DONE.mkdir(parents=True, exist_ok=True)
    for _ in range(2):
        threading.Thread(target=worker, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    log.info("listening on :%d inbox=%s dry_run=%s every_n=%d",
             HTTP_PORT, INBOX, DRY_RUN, EVERY_N)
    srv.serve_forever()


if __name__ == "__main__":
    main()
