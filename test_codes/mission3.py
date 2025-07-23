from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import cv2
from ultralytics import YOLO  
from pymavlink import mavutil

# Connect to the drone (e.g., SITL or telemetry)
vehicle = connect("tcp:127.0.0.1:5763", wait_ready=True)

# Camera specs (in mm and px — tune these)
sensor_width_mm = 7.6      # mm (horizontal sensor size)
focal_length_mm = 4.4      # mm
altitude_m = 10            # will be updated when flying

# Arm and takeoff
def arm_and_takeoff(aTargetAltitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        #print(f" Altitude: {alt:.2f}")
        if alt >= aTargetAltitude * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

# Convert pixel offset (meters) to GPS location
def get_target_location(current_location, offset_x_m, offset_y_m):
    R = 6378137.0  # Earth radius in meters
    dLat = offset_y_m / R
    dLon = offset_x_m / (R * math.cos(math.pi * current_location.lat / 180.0))       
    newlat = current_location.lat + (dLat * 180 / math.pi)
    newlon = current_location.lon + (dLon * 180 / math.pi)
    lat_long_buffer = []

    if not lat_long_buffer:
        lat_long_buffer.append((newlat,newlon))
    elif (newlat,newlon) in lat_long_buffer:
        print("ALready marked object!!")
    

    return LocationGlobalRelative(newlat, newlon, current_location.alt)

def get_distance_meters(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlon * 1.113195e5)**2)

# Load YOLO model
model = YOLO('c:/Users/Hemhalatha V R/Downloads/best_model_in_the_world.pt')

# Start mission
arm_and_takeoff(altitude_m)

# Initialize camera
cap = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')

def detect(n):
    ret, frame = cap.read()
    if not ret:
        print("cant capture image")
        return

    image_width_px = frame.shape[1] #(height,width)
    image_height_px = frame.shape[0]

    # GSD: meters per pixel
    GSD = (sensor_width_mm * vehicle.location.global_relative_frame.alt) / (focal_length_mm * image_width_px)
    
    results = model(frame)
    if results and results[0].boxes:
        detected=True
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

        print(f"Offset (meters): X = {offset_x_m:.2f}, Y = {offset_y_m:.2f}")

        # Move drone
        current_location = vehicle.location.global_relative_frame
        target_location = get_target_location(current_location, offset_x_m, offset_y_m)

        print("Navigating to target "+n)
        vehicle.simple_goto(target_location)
        while True:
            current_loc = vehicle.location.global_relative_frame
            distance = get_distance_meters(current_loc, target_location)
            if distance < 0.8:
                time.sleep(10)
                break
            time.sleep(1)

detected=False

while not detected:
    detect(1)
detect(2)
detect(3)

print("Landing...")
vehicle.mode = VehicleMode("LAND")

cap.release()
vehicle.close()