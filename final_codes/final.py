from pymavlink import mavutil
from ultralytics import YOLO
import cv2, math, time, os, keyboard
from collections import defaultdict

print("IMPORTS DONE!...JUST A MINUTE LET ME CONNECT!")

MAVLINK_CONN = "udp:127.0.0.1:14551"  # Check out
YOLO_MODEL = "best(S6).pt"
LAST_WAYPOINT = (12.8581244,77.4414401) # need to place the drop point
TARGET_ALT = 8.0
SENSOR_WIDTH_MM = 7.6/1000.0
FOCAL_LENGTH_MM = 4.4/1000.0

connection_port = 'COM18'   # Change this to your actual port
baud_rate = 57600
SERVO_CHANNEL = 10          # AUX2 = channel 10
PWM_ON = 1900               # activate / release payload
PWM_OFF = 1100              # reset position
TRIGGER_DELAY = 3.0    



object_counts = defaultdict(int)
flag = False  
DROP_COOLDOWN = 20   
last_drop_time = 0


print("🔌 Connecting to vehicle...")
vehicle = mavutil.mavlink_connection(MAVLINK_CONN)  
vehicle.wait_heartbeat()
print("✅ Connected to system (System ID:", vehicle.target_system, ")")

# Request some streams (optional)
vehicle.mav.request_data_stream_send(
    vehicle.target_system, vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    2, 1
)

model = YOLO(YOLO_MODEL)


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Error: Could not open camera.")
    exit()

os.makedirs("detections", exist_ok=True)
os.makedirs("For_Verification", exist_ok=True)



def recv_global_position_int(timeout=1):
    return vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)


def get_current_location():
    """Return (lat, lon, alt) or None"""
    msg = recv_global_position_int(timeout=1)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000.0
        return lat, lon, alt
    return None

def get_current_altitude():
    cur = get_current_location()
    return cur[2] if cur else TARGET_ALT

def get_heading():
    # Try VFR_HUD first (heading in degrees), fallback to GLOBAL_POSITION_INT.hdg if present
    msg = vehicle.recv_match(type="VFR_HUD", blocking=True, timeout=0.5)
    if msg and hasattr(msg, "heading"):
        return float(msg.heading)
    msg2 = recv_global_position_int(timeout=0.5)
    if msg2 and hasattr(msg2, "hdg"):
        return float(msg2.hdg) / 100.0
    return 0.0

def mode_set_and_wait(mode_name, timeout=6):
    """Set mode by mapping name -> id and wait for heartbeat reflecting the mode."""
    mode_map = vehicle.mode_mapping()  # returns mode names and id 
    if mode_name not in mode_map:
        print(f"❌ Mode {mode_name} not supported!")
        return False
    mode_id = mode_map[mode_name]
    vehicle.mav.set_mode_send(vehicle.target_system,
                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                              mode_id)
    start = time.time()
    while time.time() - start < timeout:
        hb = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not hb:
            continue
        if mavutil.mode_string_v10(hb) == mode_name:
            print(f"✅ Mode changed to {mode_name}")
            return True
    print(f"⚠ Mode change to {mode_name} timed out.")
    return False

def goto_location(lat, lon, alt):
    """Send position target (approx equivalent to simple_goto)."""
    vehicle.mav.set_position_target_global_int_send(
        0, vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),  # position only
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,  # velocity
        0, 0, 0,  # accel
        0, 0      # yaw, yaw_rate
    )

def get_distance_meters(loc1, loc2):
    """loc1 and loc2 are tuples (lat, lon, alt) or Location-like objects."""
    lat1, lon1 = (loc1[0], loc1[1]) if isinstance(loc1, tuple) else (loc1.lat, loc1.lon)
    lat2, lon2 = (loc2[0], loc2[1]) if isinstance(loc2, tuple) else (loc2.lat, loc2.lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)

def get_target_location(current_location, offset_east_m, offset_north_m):
    """Keep same signature as DroneKit version but current_location should be tuple (lat,lon,alt)."""
    R = 6378137.0
    dLat = offset_north_m / R
    dLon = offset_east_m / (R * math.cos(math.pi * current_location[0] / 180.0))
    newlat = current_location[0] + (dLat * 180 / math.pi)
    newlon = current_location[1] + (dLon * 180 / math.pi)
    return (newlat, newlon, current_location[2])

def camera_to_uav(x_cam, y_cam):
    return -y_cam, x_cam

def uav_to_ne(x_uav, y_uav, yaw_rad):
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east = x_uav * s + y_uav * c
    return north, east

def compute_target_location(object_centerx, object_centery, frame_width, frame_height, vehicle_conn):
    """Return (lat, lon, alt) target computed from pixel coords. vehicle_conn is MAVLink connection."""
    image_centerx = frame_width // 2
    image_centery = frame_height // 2
    altitude_m = get_current_altitude()
    GSD = (SENSOR_WIDTH_MM * altitude_m) / (FOCAL_LENGTH_MM * frame_width)

    x_cam = (object_centerx - image_centerx) * GSD
    y_cam = (object_centery - image_centery) * GSD

    x_uav, y_uav = camera_to_uav(x_cam, y_cam)
    yaw_rad = math.radians(get_heading())
    north_offset, east_offset = uav_to_ne(x_uav, y_uav, yaw_rad)

    current_location = get_current_location()
    if current_location is None:
        raise RuntimeError("No GPS available to compute target location.")
    return get_target_location(current_location, east_offset, north_offset)


def has_reached_last_waypoint(vehicle_conn, last_lat, last_lon, threshold=1.0):
    """
    Returns True if drone is within 'threshold' meters of the last waypoint.
    """
    msg = vehicle_conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    if not msg:
        return False
    current_lat = msg.lat / 1e7
    current_lon = msg.lon / 1e7

    dlat = last_lat - current_lat
    dlon = last_lon - current_lon
    dist = math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)

    if dist <= threshold:
        print(f"\n🏁 Drone reached last waypoint (within {dist:.2f} m).")
        return True
    return False


def trigger_servo(vehicle, channel, pwm_on_value, pwm_off_value, delay):
    """Activate and reset servo via MAVLink command."""
    # --- Trigger ON ---
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm_on_value,
        0, 0, 0, 0, 0
    )
    print(f"✅ Payload servo {channel} activated (PWM {pwm_on_value})")
    time.sleep(delay)

    # --- Trigger OFF ---
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm_off_value,
        0, 0, 0, 0, 0
    )
    print(f"✅ Servo {channel} reset (PWM {pwm_off_value})")
    time.sleep(2)

def payload_drop(vehicle_conn, cap, model, box, frame):
    """
    vehicle_conn : MAVLink connection (we kept name to match DroneKit code)
    cap, model, box, frame : same as DroneKit version
    Returns True if verified (simulated drop), else False
    """
    dropped = False

    x1, y1, x2, y2 = map(int, box.xyxy[0])
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    h, w = frame.shape[:2]

    # compute target lat/lon/alt
    target_lat, target_lon, target_alt = compute_target_location(cx, cy, w, h, vehicle_conn)
    print(f"📍 Target GPS: {target_lat:.7f}, {target_lon:.7f}")

    # Switch to GUIDED (using same function)
    print("🕹 Switching to GUIDED mode...")
    if not mode_set_and_wait("GUIDED", timeout=6):
        print("⚠ Failed to enter GUIDED, aborting payload approach.")
        return False

    # Send goto (similar to simple_goto)
    print("✈ Navigating toward target...")
    goto_location(target_lat, target_lon, target_alt)

    # Wait until within threshold (0.8 m to match your DroneKit)
    while True:
        cur = get_current_location()
        if cur is None:
            time.sleep(0.2)
            continue
        dist = get_distance_meters(cur, (target_lat, target_lon, target_alt))
        print(f"Distance to target: {dist:.2f} m", end="\r")
        if dist < 0.8:
            print("\n✅ Object reached.")
            break
        time.sleep(0.5)

    # Visual verification (same approach as DroneKit)
    print("🔍 Verifying pool presence...")
    verify_start = time.time()
    verified = False
    pool_center = None

    corrected_lat = None
    corrected_lon = None
    corrected_alt = None

    while time.time() - verify_start < 5:
        ret, vframe = cap.read()
        if not ret:
            continue
        results = model(vframe, conf=0.5)
        annotated = vframe.copy()
        found_pool = False
        if results and results[0].boxes:
            for box2 in results[0].boxes:
                cls_id = int(box2.cls[0])
                label = model.names[cls_id]
                conf = float(box2.conf[0]) 

                
                bx1, by1, bx2, by2 = map(int, box2.xyxy[0])
                cv2.rectangle(annotated, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                cv2.putText(annotated, f"{label} {conf:.2f}", (bx1, by1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                

                if label.upper() == "POOL":
                    found_pool = True
                    pool_center = ((bx1 + bx2)//2, (by1 + by2)//2)

        cv2.imshow("Drone Camera Feed", annotated)
        cv2.imwrite(f"For_Verification/detect_{int(time.time())}.jpg", annotated)
        cv2.waitKey(1)

        if found_pool:
            print("🎯 Pool verified — dropping payload (simulated).")
             
            corrected_lat, corrected_lon, corrected_alt = compute_target_location(
                pool_center[0], pool_center[1], vframe.shape[1], vframe.shape[0], vehicle_conn
            )
   
            
            verified = True
            break
        else:
            print("pool not present")

    if corrected_lat is None:
        print("❌ Could not verify pool again.")
        return False

    # --- Fly to corrected target before dropping ---
    print("✈ Flying to corrected drop position...")
    goto_location(corrected_lat, corrected_lon, corrected_alt)

    # Wait until center reached (<0.5m)
    while True:
        cur = get_current_location()
        if cur is None:
            time.sleep(0.2)
            continue
        dist = get_distance_meters(cur, (corrected_lat, corrected_lon, corrected_alt))
        print(f"Corrected distance: {dist:.2f}m", end="\r")
        if dist < 0.5:
            print("\n🎯 Corrected center reached.")
            break
        time.sleep(0.4)

    DROP_ALT=5
    
    # Wait until drop altitude is reached
    while True:
        cur = get_current_location()
        if cur is None:
            time.sleep(0.2)
            continue
        current_alt = cur[2]
        alt_diff = abs(current_alt - DROP_ALT)
        print(f"Altitude: {current_alt:.2f}m (target: {DROP_ALT}m, diff: {alt_diff:.2f}m)", end="\r")
        if alt_diff < 0.3:  # Within 30cm of target altitude
            print(f"\n✅ Drop altitude reached: {current_alt:.2f}m")
            break
        time.sleep(0.3)

    trigger_servo(
        vehicle_conn,
        channel=10,
        pwm_on_value=1900,
        pwm_off_value=1100,
        delay=3.0
    )
    dropped = True
    
    print("🔁 Switching to AUTO mode...")
    if not mode_set_and_wait("AUTO", timeout=6):
        print("⚠ Could not switch back to AUTO.")
    else:
        print("✅ AUTO mode restored.")

    dropped = verified
    return dropped

###### MAIN LOOP
#from ByteTrack.yolox.tracker.byte_tracker import BYTETracker

# CONFIDENCE_THRESHOLD = 0.65

# class TrackerArgs:
#     track_thresh = 0.3
#     match_thresh = 0.9
#     track_buffer = 60
#     mot20 = False

# tracker_args = TrackerArgs()
# tracker = BYTETracker(tracker_args, frame_rate=30)
                      
print("🎥 Starting camera feed and object detection...")
while True:
    if keyboard.is_pressed("q"):
        print("🛑 Stopping feed (keyboard interrupt).")
        break

    if has_reached_last_waypoint(vehicle, LAST_WAYPOINT[0], LAST_WAYPOINT[1]):
        print("✅ Drone reached final waypoint, stopping detection.")
        if flag == True:
            mode_set_and_wait("RTL", timeout=6)
        break
        
    

    ret, frame = cap.read()
    if not ret:
        print("⚠ Frame not received, retrying...")
        time.sleep(0.1)
        continue

    results = model(frame, conf=0.5)
    annotated = frame.copy()

    pool_detected = False  
    first_box = None

    
    if results and results[0].boxes:
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0]) 
          
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"{label} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            object_counts[label] += 1

            
            if label.upper() == "POOL":
                pool_detected = True
                if first_box is None:
                    first_box = box

        # Save only once per frame
        filename = f"detections/detect_{int(time.time())}.jpg"
        cv2.imwrite(filename, annotated)
        print(f"🎯 Frame processed → {filename}")

    
        if pool_detected and not flag:
            print("🏊 Pool detected — initiating payload drop...")
            flag = True
            last_drop_time = time.time()
            drop_success = payload_drop(vehicle, cap, model, first_box, frame)

        # ⏱ cooldown reset
            if flag and (time.time() - last_drop_time > DROP_COOLDOWN):
                flag = False


    # Show annotated frame
    cv2.imshow("Drone Camera Feed", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
vehicle.close()
