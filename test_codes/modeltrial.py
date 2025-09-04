print("code running..")

import cv2
from ultralytics import YOLO

print("Imports done")

model = YOLO('hyp_model.pt')

print("model imported")

cap1 = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')

print("connected")

cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)



while cap1.isOpened():
    ret, frame = cap1.read()
    if not ret:
        print("cant capture image")
        break
    
    h, w, _ = frame.shape
    print(f"Actual stream resolution: {w} x {h}")
    #small_frame = cv2.resize(frame, (640, 480))
    results = model(frame)


    width  = cap1.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap1.get(cv2.CAP_PROP_FRAME_HEIGHT)
    
    
    if results and results[0].boxes is not None and len(results[0].boxes) > 0:
        annotated_frame = results[0].plot()
        cv2.imshow('frame', annotated_frame)
    else:  
        cv2.imshow('frame', frame)
    
    print(f"Resolution: {int(width)} x {int(height)}")
    
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap1.release()
cv2.destroyAllWindows()

#as it is in conda env do the following to run it properly
'''
conda activate UAV
python modeltrial.py
'''