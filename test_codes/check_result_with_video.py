from ultralytics import YOLO
import cv2
import time
import os

def main():
    model_path = "best(S6).pt"
    video_path = "Autonomous_fly.mp4"  #<---VIDEO PATH

    # --- Check if files exist ---
    if not os.path.exists(model_path):
        print(f"❌ Model file not found: {model_path}")
        return
    if not os.path.exists(video_path):
        print(f"❌ Video file not found: {video_path}")
        return

    # --- Load YOLO model ---
    model = YOLO(model_path)

    # --- Load local video ---
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("❌ Could not open video.")
        return

    print("✅ Running YOLO video inference... (Press 'q' to quit)")

    # --- Main loop ---
    while True:
        ret, frame = cap.read()
        if not ret:
            print("✅ Video ended.")
            break

        # --- YOLO Inference ---
        results = model(frame, conf=0.5, verbose=False)
        annotated_frame = results[0].plot()

        # --- Show frame ---
        cv2.imshow("YOLO Detection", annotated_frame)

        # --- Break if 'q' pressed ---
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("👋 Exiting.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


