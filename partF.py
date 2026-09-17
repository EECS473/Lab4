"""
Part F: Object detection and tracking with YOLO  (YOLO26n via Ultralytics)

    Uses a small YOLO model (yolo26n) to detect real objects
    and send a driving command for your robot.

    Runs the model with the Ultralytics library -- the standard YOLO API:
        model = YOLO("yolo26n.pt")
        results = model(frame)
    Ultralytics handles preprocessing, NMS, and scaling boxes back to the
    frame size for you, so the detection code stays short.

    Ultralytics YOLO26 Overview: https://docs.ultralytics.com/models/yolo26#overview

    EECS 473 Lab 4 -- YOLO version
"""

# import the necessary packages
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

# ===========================================================================
# TODO: Step 1 - What object should the robot look for?
# (2 tasks)
# ===========================================================================
TARGET = ""                   # TODO: Change target to the object you want to follow
CONF = 0                      # TODO: Decide minimum confidence level (0~1)
IMGSZ = 320                   # size the model runs at (smaller = faster, less accurate)

# Load the YOLO model. If yolo26n.pt isn't already in this folder, Ultralytics
# downloads it (~5 MB) from GitHub on first run -- so the first run needs internet.
print("[INFO] loading YOLO model...")
model = YOLO("yolo26n.pt")

# model.names maps a class id -> its name (the 80 COCO classes yolo26n was
# trained on; see https://cocodataset.org/#home).
class_names = model.names

# Make sure TARGET is a known object
if TARGET not in class_names.values():
    print(f"'{TARGET}' is not a known object. Pick one of:")
    print(sorted(class_names.values()))
    raise SystemExit
target_id = next(i for i, name in class_names.items() if name == TARGET)

# Open the camera (CSI Raspberry Pi camera, via Picamera2).
# Is your Pi is constantly rebooting?
# --> Find a better power supply (5V 1.5A+)
print("[INFO] opening camera...")
picam2 = Picamera2()
# "RGB888" gives a BGR-ordered array, which is what OpenCV and Ultralytics
# expect -- so no color conversion is needed. (If colors look swapped in the
# stream, change this to "BGR888".)
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}))
picam2.start()
time.sleep(2.0)                     # let auto exposure / white balance settle

# ---- Stream the annotated video over the network ---------------------------
# Instead of cv2.imshow (which needs an HDMI monitor, and is painfully slow
# over `ssh -X` or RaspberryPi Connect), we serve the frames as MJPEG over
# HTTP. The model loop never blocks on the network: it just drops the newest
# JPEG into `latest_jpeg`, and the web server hands that to any browser that
# connects.
latest_jpeg = None              # most recent annotated frame, set by the main loop
STREAM_PORT = 8000

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
        pass                    # silence the per-request HTTP logging

threading.Thread(
    target=lambda: HTTPServer(("0.0.0.0", STREAM_PORT), MJPEGHandler).serve_forever(),
    daemon=True,
).start()
print(f"[INFO] streaming video on port {STREAM_PORT}. On your laptop (same MWireless"
      f" network as the Pi), open  http://<pi-ip>:{STREAM_PORT}/  -- get <pi-ip> from `hostname -I`")

# Count frames and time the run so we can report the average FPS at the end.
frame_count = 0
start_time = time.time()

# Loop over the frames from the video stream (press Ctrl-C in this terminal to stop)
try:
    while True:
        # Grab the next frame from the camera
        frame = picam2.capture_array()      # numpy array, BGR-ordered
        h, w, _ = frame.shape

        # ---- Run YOLO on the frame -------------------------------------------
        # Ultralytics does the preprocessing for us. We ask it to keep only our
        # TARGET class (classes=[target_id]) at CONF+ confidence, so `boxes`
        # holds just the detections we care about, in frame-pixel coordinates.
        results = model.predict(frame, imgsz=IMGSZ, conf=CONF,
                                classes=[target_id], verbose=False)
        boxes = results[0].boxes

        # ---- Find the best TARGET detection ----------------------------------
        center = None
        if len(boxes) > 0:
            best = boxes[int(boxes.conf.argmax())]      # highest-confidence one
            x1, y1, x2, y2 = best.xyxy[0].tolist()      # already in frame pixels
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # ===========================================================================
        # TODO: Step 2 - Write your robot command
        # (3 tasks)
        # ===========================================================================
        if center is not None:             # = if TARGET object is found
            cx, cy = center
            print(f"[INFO] {TARGET} found at x={cx}, y={cy}")

            # TODO: Write your robot control logic. Be creative!
            # HINT: The frame is `w` pixels wide, so its middle is w // 2.
                # e.g. if {cx} is less than {w//2}, which side is the object on?

            if cx < w // 2 - 50:
                command = "LEFT"
            elif cx > w // 2 + 50:
                command = "RIGHT"
            else:
                command = "FORWARD"

            print("[INFO] robot command:", command)
            # TODO: Actually send `command` to your robot


        else:
            print(f"[INFO] {TARGET} not in view")
            # TODO: Decide what the robot should do when it can't see the object

        # Publish the annotated frame to the MJPEG stream (replaces cv2.imshow).
        # Encoding to JPEG is cheap next to YOLO inference, so the loop stays fast.
        ok_jpg, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok_jpg:
            latest_jpeg = buf.tobytes()

        frame_count += 1

except KeyboardInterrupt:
    pass                                   # Ctrl-C -> fall through to cleanup

# cleanup
print("\n [INFO] Exiting Program and cleanup stuff \n")
elapsed = time.time() - start_time
if elapsed > 0 and frame_count > 0:
    print(f"[INFO] processed {frame_count} frames in {elapsed:.1f}s "
          f"-> {frame_count / elapsed:.2f} FPS")
picam2.stop()
