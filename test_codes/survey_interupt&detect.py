print("Survey + Detection code running...")

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import cv2
from ultralytics import YOLO  

print("Imports Done")

# Connect to the drone
print("Searching for VAMOS")
vehicle = connect("COM10", wait_ready=True, baud=57600)
print("Drone connected....")

# Camera specs (in mm and px — tune these)
sensor_width_mm = 7.6    # mm (horizontal sensor size)
focal_length_mm = 4.6    # mm

# Detection settings
DETECTION_CONFIDENCE = 0.5
MAX_TRACKING_DETECTIONS = 5
DESCENT_PER_DETECTION = 1.0  # Descend 1m per detection
MIN_SAFE_ALTITUDE = 3.0  # Minimum altitude in meters

def get_target_location(current_location, north_offset, east_offset, alt):
    """Convert pixel offset (meters) to GPS location"""
    R = 6378137.0  # Earth radius in meters

    dLat = north_offset / R
    dLon = east_offset / (R * math.cos(math.pi * current_location.lat / 180.0))

    newlat = current_location.lat + (dLat * 180 / math.pi)
    newlon = current_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, alt)

def get_distance_meters(loc1, loc2):
    """Calculate distance between two GPS coordinates"""
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlon * 1.113195e5)**2)

def camera_to_uav(x_cam, y_cam):
    """Convert camera frame to UAV body frame"""
    x_uav = -y_cam  
    y_uav = x_cam   
    return x_uav, y_uav

def uav_to_ne(x_uav, y_uav, yaw_rad):
    """Convert UAV body frame to North-East frame"""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east = x_uav * s + y_uav * c
    return north, east

def cancel_mission():
    """Cancel current mission and switch to GUIDED mode"""
    print("\n🛑 CANCELING SURVEY MISSION...")
    
    # Clear mission
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()
    time.sleep(1)
    
    # Switch to GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    
    # Wait for mode change
    while vehicle.mode.name != "GUIDED":
        print(" Waiting for GUIDED mode...")
        time.sleep(0.5)
    
    print("✅ Mission canceled. Now in GUIDED mode.")

def get_mission_status():
    """Check if mission is active"""
    # Mission is active if in AUTO mode and has waypoints
    if vehicle.mode.name == "AUTO":
        nextwaypoint = vehicle.commands.next
        num_waypoints = vehicle.commands.count
        return True, nextwaypoint, num_waypoints
    return False, 0, 0

def track_object(frame, model, planned_altitude, detection_count):
    """Detect and track object, return new altitude and detection count"""
    
    image_width_px = frame.shape[1]
    image_height_px = frame.shape[0]

    # Get current altitude for GSD calculation
    current_altitude = vehicle.location.global_relative_frame.alt
    if current_altitude is None or current_altitude < 1:
        current_altitude = planned_altitude

    # GSD: meters per pixel
    GSD = (sensor_width_mm * current_altitude) / (focal_length_mm * image_width_px)
    
    # Run YOLO detection
    results = model(frame, conf=DETECTION_CONFIDENCE, verbose=False)

    # Check if any objects detected
    if results and results[0].boxes is not None and len(results[0].boxes) > 0:
        annotated_frame = results[0].plot() 
        cv2.imshow('Detection Feed', annotated_frame)
        
        # Save detection image
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(f"survey_detection_{timestamp}.jpg", annotated_frame)
        
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

        # Calculate new altitude using PLANNED altitude
        new_altitude = planned_altitude - DESCENT_PER_DETECTION
        if new_altitude < MIN_SAFE_ALTITUDE:
            new_altitude = MIN_SAFE_ALTITUDE
            print(f"⚠️ WARNING: Minimum safe altitude reached ({MIN_SAFE_ALTITUDE}m)")

        print(f"\n=== Detection {detection_count + 1}/{MAX_TRACKING_DETECTIONS} ===")
        print(f"Planned altitude: {planned_altitude:.2f}m → New target: {new_altitude:.2f}m")
        print(f"Actual current altitude: {current_altitude:.2f}m")
        print(f"Pixel offset: X={object_centerx - image_centerx}, Y={object_centery - image_centery}")
        print(f"Real offset (m): X={offset_x_m:.2f}, Y={offset_y_m:.2f}")
        print(f"North-East offset (m): N={north_offset:.2f}, E={east_offset:.2f}")

        # Get current location and calculate target
        current_location = vehicle.location.global_relative_frame
        target_location = get_target_location(
            current_location, 
            north_offset, 
            east_offset, 
            new_altitude
        )
        
        print(f"Current: Lat={current_location.lat:.7f}, Lon={current_location.lon:.7f}")
        print(f"Target: Lat={target_location.lat:.7f}, Lon={target_location.lon:.7f}")
        
        distance = get_distance_meters(current_location, target_location)
        print(f"Distance to target: {distance:.2f} m")

        # FAILSAFE: Don't move more than 50m in one command during survey
        if distance > 50:
            print(f"⚠️ SAFETY: Target too far ({distance:.2f}m)! Skipping movement.")
            return planned_altitude, detection_count, False

        # Move towards target
        if distance > 0.5:
            print("Moving towards target and descending...")
            vehicle.simple_goto(target_location)
            time.sleep(5)
            
            actual_alt = vehicle.location.global_relative_frame.alt
            print(f"Movement complete. Actual altitude: {actual_alt:.2f}m")
        else:
            print("Target very close, only descending...")
            descend_location = LocationGlobalRelative(
                current_location.lat,
                current_location.lon,
                new_altitude
            )
            vehicle.simple_goto(descend_location)
            time.sleep(5)
        
        return new_altitude, detection_count + 1, True
    
    else:
        # No detection
        cv2.imshow('Detection Feed', frame)
        return planned_altitude, detection_count, False

# Load YOLO model
model = YOLO('best.pt')
print("Model Loaded!")

video_input = 'rtsp://192.168.144.25:8554/main.264'

# Initialize camera
cap = cv2.VideoCapture(video_input)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("Error: Cannot open video stream")
    vehicle.close()
    exit()

print("Feed received!")
print("\n" + "="*50)
print("MISSION MODE: Survey will run until object detected")
print("Upload your survey mission in Mission Planner")
print("Set vehicle to AUTO mode to start survey")
print("="*50 + "\n")

# Wait for mission to be uploaded and AUTO mode
print("Waiting for survey mission to start (AUTO mode)...")
while vehicle.mode.name != "AUTO":
    print(f" Current mode: {vehicle.mode.name} - Switch to AUTO to start survey")
    time.sleep(2)

print("\n✅ SURVEY MISSION STARTED!")
print("Monitoring video feed for target object...\n")

# Mission monitoring variables
mission_active = True
detection_count = 0
planned_altitude = None
tracking_mode = False

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Can't capture image")
            break

        # Check mission status
        if not tracking_mode:
            is_mission_active, current_wp, total_wp = get_mission_status()
            
            if is_mission_active:
                print(f"Survey progress: Waypoint {current_wp}/{total_wp}", end='\r')
                
                # Check for object detection during survey
                results = model(frame, conf=DETECTION_CONFIDENCE, verbose=False)
                
                if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                    print("\n\n🎯 OBJECT DETECTED DURING SURVEY!")
                    
                    # Cancel mission
                    cancel_mission()
                    
                    # Get current altitude for tracking
                    planned_altitude = vehicle.location.global_relative_frame.alt
                    
                    # Switch to tracking mode
                    tracking_mode = True
                    print(f"\n🔄 SWITCHING TO TRACKING MODE")
                    print(f"Starting altitude: {planned_altitude:.2f}m")
                    print(f"Will track for {MAX_TRACKING_DETECTIONS} detections\n")
                    time.sleep(2)
                else:
                    # Show normal feed during survey
                    cv2.imshow('Detection Feed', frame)
            else:
                # Mission completed or not active
                print("\nSurvey mission completed or inactive.")
                break
        
        else:
            # TRACKING MODE - Object detected, mission canceled
            if detection_count < MAX_TRACKING_DETECTIONS:
                planned_altitude, detection_count, detected = track_object(
                    frame, model, planned_altitude, detection_count
                )
                
                if not detected:
                    print("Object lost during tracking. Searching...")
                    time.sleep(1)
            else:
                print(f"\n✅ TRACKING COMPLETE ({MAX_TRACKING_DETECTIONS} detections done)")
                print(f"Final altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
                break

        # Check for quit command
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nUser requested quit")
            break

except KeyboardInterrupt:
    print("\n\nInterrupted by user")

finally:
    print("\n" + "="*50)
    print("LANDING SEQUENCE")
    print("="*50)
    
    # Land the drone
    vehicle.mode = VehicleMode("LAND")
    
    # Wait for landing
    while vehicle.armed:
        current_alt = vehicle.location.global_relative_frame.alt
        if current_alt is not None:
            print(f" Landing... altitude: {current_alt:.2f}m")
        time.sleep(1)
    
    print("✅ Landed successfully")
    
    # Cleanup
    cv2.destroyAllWindows()
    cap.release()
    vehicle.close()
    print("Program ended")