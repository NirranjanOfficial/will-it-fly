print("Code runnin.....")

import cv2
from ultralytics import YOLO

print("Imports done.....")

# -----------------------------
# CONFIGURATION
# -----------------------------
RTSP_URL = "rtsp://192.168.144.25:8554/main.264"  # <-- replace with your stream
MODEL_PATH = "best.pt"  # You can use yolov8n.pt, yolov8s.pt, yolov8m.pt, etc.


print("Parts passed.....")
# -----------------------------
# LOAD MODEL
# -----------------------------
model = YOLO(MODEL_PATH)
print("Model loaded....")
# -----------------------------
# OPEN RTSP STREAM
# -----------------------------
cap = cv2.VideoCapture(RTSP_URL)
print("Captured.....")
if not cap.isOpened():
    print("❌ Cannot open RTSP stream.")
    exit()

print("✅ RTSP stream connected. Running YOLO inference...")

# -----------------------------
# LOOP THROUGH FRAMES
# -----------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Failed to grab frame. Reconnecting...")
        break

    # Run YOLO inference on the frame
    results = model(frame, verbose=False)

    # Draw detections on the frame
    annotated_frame = results[0].plot()

    # Display output
    cv2.imshow("YOLO RTSP Inference", annotated_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# -----------------------------
# CLEANUP
# -----------------------------
cap.release()
cv2.destroyAllWindows()
