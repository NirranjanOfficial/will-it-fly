import cv2
import time
import json
from ultralytics import YOLO


MODEL_PATH = "best(h).pt"   # ✅ use your model
VIDEO_SOURCE = "testVideo.mp4"   # or 0 for webcam

model = YOLO(MODEL_PATH)

MIN_CONF = 0.3

# ===========================
# MAIN
# ===========================
def main():

    cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print("❌ Unable to open video source")
        return

    frame_count = 0
    start_time = time.time()

    # We store count per Track-ID
    object_registry = {}
    class_counts = {}

    print("\n✅ ByteTrack + YOLO tracking started...\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        H, W = frame.shape[:2]

        # ✅ YOLO + ByteTrack
        results = model.track(
            frame,
            imgsz=640,
            conf=MIN_CONF,
            persist=True,
            tracker="bytetrack.yaml"   # ✅ built-in ByteTrack
        )

        detections = results[0]

        if detections.boxes is not None:
            for box in detections.boxes:
                # ✅ TRACK ID
                if box.id is None:
                    continue  # skip untracked objects
                track_id = int(box.id[0])

                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # ✅ Unique counting
                if track_id not in object_registry:
                    object_registry[track_id] = cls_name
                    class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

                # ✅ Draw tracking info
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame, f"{cls_name} ID:{track_id}", (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

        # ✅ live overlay of counts
        y = 20
        for cname, count in class_counts.items():
            cv2.putText(frame, f"{cname}: {count}", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            y += 20

        cv2.imshow("YOLO + ByteTrack", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # ===========================
    # END — performance print
    # ===========================
    total_time = time.time() - start_time
    fps = frame_count / total_time if total_time > 0 else 0

    print("\n✅ DONE")
    print(f"Resolution: {W} x {H}")
    print(f"Total Frames: {frame_count}")
    print(f"Average FPS: {fps:.2f}")

    print("\n✅ Final Object Counts:")
    print(json.dumps(class_counts, indent=2))

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
