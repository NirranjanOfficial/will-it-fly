print("code running..")

from threading import Thread,Lock
import time
import numpy as np
import cv2
from ultralytics import YOLO

print("Imports done")

model = YOLO('hyp_model.pt')
print("model imported")

cap = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')
print("connected")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  #changed the resolution
cap.set(cv2.CAP_PROP_BUFFERSIZE,3)   #just in case the feed lags im setting up an buffer of 3 here

new_frame = None
frame_lock = Lock()
running = True

def frame_reader():
    global new_frame, running
    frame_counter = 0
    prev_time = time.time()
    while running:
        ret, frame = cap.read()
        if not ret:
            print("cant capture...trying to!")
            continue
        with frame_lock:
            new_frame = frame.copy()
        frame_counter += 1
        current_time = time.time()
        if current_time - prev_time>=1.0:
            h,w,_ = frame.shape
            print(f"Incoming RTSP FPS: {frame_counter} and Resolution: {w} x {h}")
            frame_counter = 0
            prev_time = current_time

        time.sleep(0.01)


def interfacing():
    global new_frame,running
    while running:
        frame = None
        with frame_lock:
            if new_frame is not None:
                frame = new_frame.copy()
        if frame is not None:
            results = model(frame)

            if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                annotated_frame = results[0].plot()
                cv2.imshow("frame", annotated_frame)
            else:
                cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False
            break


interface_thread = Thread(target=interfacing, daemon=True)
reading_thread = Thread(target=frame_reader,daemon=True)

reading_thread.start()
interface_thread.start()
interface_thread.join()

cap.release()
cv2.destroyAllWindows()

#as it is in conda env do the following to run it properly
'''
conda activate UAV
python modeltrial.py
'''