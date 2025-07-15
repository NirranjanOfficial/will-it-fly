<<<<<<< HEAD
from dronekit import connect
import cv2 as cv
import math
import time

vehicle = connect('<COM PORT>', baud='< baudrate as int>', wait_ready=True)

if vehicle is not None:
    print("Kandupidiciten!!")
if vehicle.is_armable == True:
    print("Vanakam da maplaaa!! Connect agiten!")

#model = 'yolov8n'

rtsp = '<url>'
capture = cv.VideoCapture(rtsp)  

###     Sensor deafult values     ###

sensor_width = 7.6 #mm
focal_length = 4.4 #mm


###     Used to continously go thorugh the input video frame        ###
while True:
    ret, frame =capture.read()
    tym = 0
    if not ret:
        print(f"no frame got grabbed!{tym} secs")
        time.sleep(1)
    
    image_height, image_width, channel = frame.shape
    alt = vehicle.location.global_relative_frame.alt
    gsd = (sensor_width*alt) / (focal_length*image_width)

    #To annotate on the Screen!
    cv.putText(frame,                      
            f"Height: {alt:.2f} m",     
            (50, 50),                   
            cv.FONT_HERSHEY_DUPLEX,    
            1.8,                        
            (0,0, 255),            
            2)
                              
    cv.putText(frame,                      
            f"GSD: {gsd:.2f} m",     
            (50, 45),                   
            cv.FONT_HERSHEY_DUPLEX,    
            1.8,                        
            (0,0, 255),            
            2)


    cv.imshow('Video',frame)
    
    #used to break the code and kill the processs!
    if cv.waitKey(20) & 0xFF==ord('x'): 
        break


capture.release() 
cv.destroyAllWindows()
=======
from dronekit import connect
import cv2 as cv
#import math
import time
from ultralytics import YOLO
from dronekit import VehicleMode
from pymavlink import mavutil

'''
###     basic ARMING code from the documantation of Dronekit API        ###

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: "), vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude:
            print ("Reached target altitude")
            break  
        time.sleep(1)

###     WITH THE ABOVE BLOCK WE CAN ONLY TAKE OFF TO CERTAIN ALTITUDE AND HOVER     ###
'''

vehicle = connect('<COM PORT>', baud='< baudrate as int>', wait_ready=True)

if vehicle is not None:
    print("Kandupidiciten!!")
if vehicle.is_armable == True:
    print("Vanakam da maplaaa!! Connect agiten!")

model = YOLO('yolov8n')

rtsp = '<url>'
capture = cv.VideoCapture(rtsp)  

###     Sensor deafult values     ###

sensor_width = 7.6 # mm
focal_length = 4.4 # mm

#ARM to fly

'''arm_and_takeoff(20)'''

###     Used to continously go thorugh the input video frame        ###

while True:
    ret, frame =capture.read()
    tym = 0
    if not ret:
        print(f"no frame got grabbed!{tym} secs")
        time.sleep(1)
        tym +=1
    feed = model(frame)

    image_height, image_width, channel = frame.shape
    image_centerx = image_width//2
    image_centery = image_height//2 
    alt = vehicle.location.global_relative_frame.alt
    gsd = (sensor_width*alt) / (focal_length*image_width)

    ###     To annotate on the Screen!      ###
    cv.putText(frame,                      
            f"Height: {alt:.2f} m",     
            (50, 50),                   
            cv.FONT_HERSHEY_DUPLEX,    
            1.8,                        
            (0,0, 255),            
            2)                         
    cv.putText(frame,                      
            f"GSD: {gsd:.2f} m",     
            (50, 45),                   
            cv.FONT_HERSHEY_DUPLEX,    
            1.8,                        
            (0,0, 255),            
            2)
    
    results = model(frame)
    annotated_frame = results[0].plot()
    boxes = results[0].boxes

    for box in boxes:
        xmin,ymin,xmax,ymax = box.xyxy[0]   # to find the center of the object in the frame
        object_centerx = (xmin+xmax)//2
        object_centery = (ymin+ymax)//2

    c2cdistance_x = (image_centerx - object_centerx) * gsd
    c2cdistance_y = (image_centery - object_centery) * gsd


    cv.imshow("YOLOv8 RTSP Detection", annotated_frame)

    #used to break the code and kill the processs!
    if cv.waitKey(20) & 0xFF==ord('x'): 
        break


capture.release() 
cv.destroyAllWindows()


'''if i knew the geo location of the object i can use goto in order to reach there!

a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)
vehicle.simple_goto(a_location,groundspeed=7.5)

'''


'''
-----------------------WORK FLOW----------------------

CONNECT TO DRONE
    |
ARM THE DRONE
    |
REACH CERTAIN HEIGHT
    |
CV FOR FRAME INPUT
    |
GSD CALCULATION
    |
IDENTIFY THE OBJECT WITHIN THE FRAME
    |
FIND THE CENTER POINT OF THE OBJECT IN THAT FRAME
    |
FIND THE CURRENT CENTER
    |
IDENTIFY THE CENTER
    |
GO TO THE CENTER OF OBJECT
    |
START TO DECEND <-----|
    |                 |
CALCULATE THE GSD ----|
    |
REACH THE CENTER
    |
LAND & DISARM

----------------------------------------------------

'''
>>>>>>> a6783f8 (Initial commit)
