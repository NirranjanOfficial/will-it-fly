print("code running...")

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import cv2
from ultralytics import YOLO  

print("Imports Done")

# Connect to the drone (e.g., SITL or telemetry)
print("Searching for VAMOS")
vehicle = connect("COM10", wait_ready=True)   #ENTER THE COM VALUE BEFORE RUNNING
print("drone connected....")
vehicle.mode = VehicleMode("GUIDED")

# Camera specs (in mm and px — tune these)
sensor_width_mm = 7.6    # mm (horizontal sensor size)
focal_length_mm = 4.6    # mm
altitude_m = 10 # will be updated when flying

def arm_and_takeoff(target_altitude):
    print("Arming motors...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Current altitude: {current_alt:.2f}")
        if current_alt >= target_altitude * 0.95:
            print("Target altitude reached!")
            break
        time.sleep(1)


# Convert pixel offset (meters) to GPS location
def get_target_location(current_location, offset_x_m, offset_y_m,alt1):
    R = 6378137.0  # Earth radius in meters

    dLat = offset_y_m / R
    dLon = offset_x_m / (R * math.cos(math.pi * current_location.lat / 180.0))

    newlat = current_location.lat + (dLat * 180 / math.pi)
    newlon = current_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, alt1)

def get_distance_meters(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlon * 1.113195e5)**2)

def camera_to_uav(x_cam, y_cam):
    x_uav = -y_cam  
    y_uav = x_cam   
    return x_uav, y_uav

def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east = x_uav * s + y_uav * c
    return north, east

# Load YOLO model
model = YOLO('best.pt')         # <---- enter the pt file path here!
print("Model Loaded..!!!")

video_input = 'rtsp://192.168.144.25:8554/main.264'


# Initialize camera
cap = cv2.VideoCapture(video_input)

print("Feed recieved....")

arm_and_takeoff(altitude_m)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("can't capture image")
        break

    image_width_px = frame.shape[1] #(height,width)
    image_height_px = frame.shape[0]

    # GSD: meters per pixel
    GSD = (sensor_width_mm * vehicle.location.global_relative_frame.alt) / (focal_length_mm * image_width_px)
    results = model(frame)
    #print("feed")

    cv2.imshow('frame',frame)
    if results and results[0].boxes is not None and len(results[0].boxes) > 0:
        annotated_frame = results[0].plot() 
        cv2.imshow('frame',annotated_frame)
        cv2.imwrite("real_test_detected.jpg", annotated_frame)
        box = results[0].boxes[0]
        
        xmin, ymin, xmax, ymax = box.xyxy[0]

        object_centerx = int((xmin + xmax) / 2)
        object_centery = int((ymin + ymax) / 2)

        # Center of the image
        image_centerx = image_width_px // 2
        image_centery = image_height_px // 2

        # Pixel offset → real-world offset (in meters)
        offset_x_m = (object_centerx - image_centerx) * GSD
        offset_y_m = (object_centery - image_centery) * GSD

        x_uav, y_uav = camera_to_uav(offset_x_m ,offset_y_m )

    # Convert to North-East using current heading
        yaw_rad = math.radians(vehicle.heading)
        north_offset, east_offset = uav_to_ne(x_uav, y_uav, yaw_rad)

        print(f"Offset (meters): X = {offset_x_m:.2f}, Y = {offset_y_m:.2f}")

        # Move drone
        print("Lets move towards the point!")

        current_location = vehicle.location.global_relative_frame
        target_location = get_target_location(current_location, north_offset, east_offset)
        print("current: x:",current_location.lat," y=",current_location.lon)
        print("target: x:",target_location.lat," y=",target_location.lon)    
        distance = get_distance_meters(current_location, target_location)
        print("distance: ",distance)

        print("Navigating to target ")

        vehicle.simple_goto(target_location)
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    

print("Landing...Ready to take manual!")

#trigger_land=input("enter any key to land: ")
vehicle.mode = VehicleMode("LAND")

cv2.destroyAllWindows()
cap.release()
vehicle.close()