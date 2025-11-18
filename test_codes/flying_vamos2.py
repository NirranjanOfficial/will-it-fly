print("code running...")

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import cv2
from ultralytics import YOLO  

print("Imports Done")

# Connect to the drone (e.g., SITL or telemetry)
print("Searching for VAMOS")
vehicle = connect("COM10", wait_ready=True, baud=57600)   # Added baud rate
print("drone connected....")
vehicle.mode = VehicleMode("GUIDED")

# Camera specs (in mm and px — tune these)
sensor_width_mm = 7.6    # mm (horizontal sensor size)
focal_length_mm = 4.6    # mm
target_altitude = 10     # meters

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
def get_target_location(current_location, north_offset, east_offset, alt):
    R = 6378137.0  # Earth radius in meters

    dLat = north_offset / R
    dLon = east_offset / (R * math.cos(math.pi * current_location.lat / 180.0))

    newlat = current_location.lat + (dLat * 180 / math.pi)
    newlon = current_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, alt)

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
model = YOLO('best.pt')  # Fixed typo: taget -> target
print("Model Loaded..!!!")

video_input = 'rtsp://192.168.144.25:8554/main.264'

# Initialize camera
cap = cv2.VideoCapture(video_input)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency

if not cap.isOpened():
    print("Error: Cannot open video stream")
    vehicle.close()
    exit()

print("Feed received....")

# Arm and takeoff
arm_and_takeoff(target_altitude)

# Add delay to stabilize after takeoff
time.sleep(2)

detection_count = 0
max_detections = 5  # Will track 5 times, descending 1m each time
descent_per_detection = 1.0  # Descend 1 meter per detection

try:
    while cap.isOpened() and detection_count < max_detections:
        ret, frame = cap.read()
        if not ret:
            print("Can't capture image")
            break

        image_width_px = frame.shape[1]
        image_height_px = frame.shape[0]

        # Get current altitude for GSD calculation
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude is None or current_altitude < 1:
            current_altitude = target_altitude  # Fallback

        # GSD: meters per pixel
        GSD = (sensor_width_mm * current_altitude) / (focal_length_mm * image_width_px)
        
        # Run YOLO detection
        results = model(frame, conf=0.5, verbose=False)

        # Check if any objects detected
        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            annotated_frame = results[0].plot() 
            cv2.imshow('Detection Feed', annotated_frame)
            
            # Save detection image
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"detection_{timestamp}.jpg", annotated_frame)
            
            # Get first detected object
            box = results[0].boxes[0]
            xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()

            object_centerx = int((xmin + xmax) / 2)
            object_centery = int((ymin + ymax) / 2)

            # Center of the image
            image_centerx = image_width_px // 2
            image_centery = image_height_px // 2

            # Pixel offset → real-world offset (in meters)
            offset_x_m = (object_centerx - image_centerx) * GSD
            offset_y_m = (object_centery - image_centery) * GSD

            # Convert camera frame to UAV frame
            x_uav, y_uav = camera_to_uav(offset_x_m, offset_y_m)

            # Convert to North-East using current heading
            yaw_rad = math.radians(vehicle.heading)
            north_offset, east_offset = uav_to_ne(x_uav, y_uav, yaw_rad)

            print(f"\n=== Detection {detection_count + 1}/{max_detections} ===")
            print(f"Current altitude: {current_altitude:.2f}m → Target altitude: {new_altitude:.2f}m")
            print(f"Pixel offset: X={object_centerx - image_centerx}, Y={object_centery - image_centery}")
            print(f"Real offset (m): X={offset_x_m:.2f}, Y={offset_y_m:.2f}")
            print(f"North-East offset (m): N={north_offset:.2f}, E={east_offset:.2f}")

            # Get current location and calculate target
            current_location = vehicle.location.global_relative_frame
            
            # Calculate new altitude (descend 1m each detection)
            new_altitude = current_altitude - descent_per_detection
            if new_altitude < 2:  # Safety: don't go below 2m
                new_altitude = 2
                print("WARNING: Minimum altitude reached (2m)")
            
            target_location = get_target_location(
                current_location, 
                north_offset, 
                east_offset, 
                new_altitude  # Descending altitude
            )
            
            print(f"Current: Lat={current_location.lat:.7f}, Lon={current_location.lon:.7f}")
            print(f"Target: Lat={target_location.lat:.7f}, Lon={target_location.lon:.7f}")
            
            distance = get_distance_meters(current_location, target_location)
            print(f"Distance to target: {distance:.2f} m")

            # Only move if distance is significant (> 0.5m to avoid jitter)
            if distance > 0.5:
                print("Moving towards target and descending...")
                vehicle.simple_goto(target_location)
                
                # Wait for drone to reach approximate position and altitude
                time.sleep(5)
                
                # Verify altitude change
                time.sleep(2)
                actual_alt = vehicle.location.global_relative_frame.alt
                print(f"Movement complete. Current altitude: {actual_alt:.2f}m")
                
            else:
                print("Target very close, only descending...")
                # Just descend without horizontal movement
                descend_location = LocationGlobalRelative(
                    current_location.lat,
                    current_location.lon,
                    new_altitude
                )
                vehicle.simple_goto(descend_location)
                time.sleep(5)
                
            detection_count += 1
            
        else:
            # No detection - show original frame
            cv2.imshow('Detection Feed', frame)

        # Check for quit command
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("User requested quit")
            break

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    print("\nLanding... Ready to take manual control!")
    vehicle.mode = VehicleMode("LAND")
    
    # Wait for landing
    while vehicle.armed:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Landing... altitude: {current_alt:.2f}")
        time.sleep(1)
    
    print("Landed successfully")
    
    # Cleanup
    cv2.destroyAllWindows()
    cap.release()
    vehicle.close()
    print("Program ended")