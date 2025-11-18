'''
WHY THIS CODE?
this code is to take manual fly and to check if the model can detect stuff!
no dronekit involved!
'''


print("code is running!")

import cv2
from ultralytics import YOLO
import torch

print("Imports are done")

# Configuration
RTSP_URL = "rtsp://192.168.144.25:8554/main.264"
MODEL_PATH = "best.pt"
CONFIDENCE_THRESHOLD = 0.7

def main():
    # Load YOLOv8 model
    print("Loading YOLOv8 model...")
    model = YOLO(MODEL_PATH)
    
    # Check if CUDA is available
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device}")
    model.to(device)
    
    # Open RTSP stream
    print(f"Connecting to RTSP stream: {RTSP_URL}")
    cap = cv2.VideoCapture(RTSP_URL)
    
    # Set buffer size to reduce latency
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    if not cap.isOpened():
        print("Error: Could not open RTSP stream")
        return
    
    print("Stream connected successfully!")
    print("Press 'q' to quit")
    
    frame_count = 0
    
    try:
        while True:
            # Read frame from stream
            ret, frame = cap.read()
            
            if not ret:
                print("Error: Failed to read frame from stream")
                break
            
            frame_count += 1
            
            # Run YOLOv8 inference
            results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)
            
            # Annotate frame with detection results
            annotated_frame = results[0].plot()
            
            # Display FPS and frame count
            fps_text = f"Frame: {frame_count}"
            cv2.putText(annotated_frame, fps_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show the frame
            cv2.imshow('YOLOv8 Drone Detection', annotated_frame)
            
            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exiting...")
                break
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        print("Stream closed")

if __name__ == "__main__":
    main()