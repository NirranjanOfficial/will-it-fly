from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import random
import cv2

# ----------------------------
# CONFIGURATION
# ----------------------------
# Connect to SITL (or your autopilot)
connection_string = '127.0.0.1:14550'  # Adjust if needed
vehicle = connect(connection_string, wait_ready=True)

# Camera setup
cap = cv2.VideoCapture(0)

# Time to wait before switching to GUIDED (seconds)
wait_before_guided = 20
guided_duration = 10  # Time to stay in GUIDED mode

# ----------------------------
# FUNCTIONS
# ----------------------------
def random_nearby_location(current_location, radius_m=5):
    """Generate a nearby random location (meters)."""
    import math

    # Approximate conversions
    dlat = radius_m / 111111  # 1 deg latitude ~ 111 km
    dlon = radius_m / (111111 * math.cos(math.radians(current_location.lat)))

    new_lat = current_location.lat + random.uniform(-dlat, dlat)
    new_lon = current_location.lon + random.uniform(-dlon, dlon)

    return LocationGlobalRelative(new_lat, new_lon, current_location.alt)

# ----------------------------
# MAIN LOOP
# ----------------------------
start_time = time.time()
guided_triggered = False
guided_start_time = None

print("Starting mission in AUTO mode...")
vehicle.mode = VehicleMode("AUTO")

try:
    while True:
        # Read camera feed
        ret, frame = cap.read()
        if ret:
            cv2.imshow("Camera Feed", frame)

        # Switch to GUIDED after wait_before_guided seconds
        elapsed = time.time() - start_time
        if not guided_triggered and elapsed > wait_before_guided:
            print("Switching to GUIDED mode...")
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
            # Fly to a random nearby point
            target = random_nearby_location(vehicle.location.global_relative_frame, radius_m=10)
            print(f"Going to random location: {target.lat}, {target.lon}")
            vehicle.simple_goto(target)
            guided_triggered = True
            guided_start_time = time.time()

        # Switch back to AUTO after guided_duration
        if guided_triggered and (time.time() - guided_start_time) > guided_duration:
            print("Switching back to AUTO mode...")
            vehicle.mode = VehicleMode("AUTO")
            guided_triggered = False  # Only do once or you can loop this

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

finally:
    print("Closing camera and vehicle connection...")
    cap.release()
    cv2.destroyAllWindows()
    vehicle.close()
