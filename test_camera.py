"""
test_camera.py -- quick check that the camera + HTTP video stream work.

No YOLO, no detection: just grab frames from the camera and serve them as an
MJPEG stream. Use this to confirm your camera and network are working before
running the full lab (partF.py).

    python test_camera.py

Then on your laptop (same MWireless network as the Pi), open
    http://<pi-ip>:8000/          (get <pi-ip> from `hostname -I` on the Pi)
Press Ctrl-C in this terminal to stop.
"""

import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
from picamera2 import Picamera2

STREAM_PORT = 8000
latest_jpeg = None                  # newest frame, set by the main loop

# ---- MJPEG HTTP server (runs in a background thread) ------------------------
class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        while True:
            if latest_jpeg is not None:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(latest_jpeg)
                self.wfile.write(b"\r\n")
            time.sleep(0.05)

    def log_message(self, *args):
        pass                        # silence per-request HTTP logging

threading.Thread(
    target=lambda: HTTPServer(("0.0.0.0", STREAM_PORT), MJPEGHandler).serve_forever(),
    daemon=True,
).start()
print(f"[INFO] streaming camera on port {STREAM_PORT}. On your laptop, open")
print(f"[INFO]   http://<pi-ip>:{STREAM_PORT}/   (get <pi-ip> from `hostname -I`)")

# ---- Open the camera (CSI Raspberry Pi camera, via Picamera2) ---------------
print("[INFO] opening camera...")
picam2 = Picamera2()
# "RGB888" gives a BGR-ordered array, which is what OpenCV expects -- so no
# color conversion is needed. (If colors look swapped, change this to "BGR888".)
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}))
picam2.start()
time.sleep(2.0)                     # let auto exposure / white balance settle

# ---- Grab frames and publish them to the stream ----------------------------
frame_count = 0
try:
    while True:
        frame = picam2.capture_array()      # numpy array, BGR-ordered
        ok_jpg, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok_jpg:
            latest_jpeg = buf.tobytes()

        # Print a heartbeat so it's clear the camera is working (this script has
        # no other per-frame output). First frame confirms it's live.
        if frame_count == 0:
            print("[INFO] camera OK -- streaming now. Open the URL above; Ctrl-C to stop.")
        frame_count += 1
        if frame_count % 100 == 0:
            print(f"[INFO] still streaming ({frame_count} frames)")
except KeyboardInterrupt:
    pass

print("\n [INFO] Exiting -- cleanup \n")
picam2.stop()
