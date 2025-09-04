from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=True)

survey_done = False
target_lat = None
target_lon = None

# Example: store object location when detected
def on_object_detected(lat, lon):
    global target_lat, target_lon
    target_lat = lat
    target_lon = lon
    print("Object detected at:", lat, lon)

# Step 1: download mission to know number of waypoints
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
total_wps = cmds.count
print("Total mission waypoints:", total_wps)

# Step 2: listen for mission progress
@vehicle.on_message('MISSION_CURRENT')
def listener(self, name, msg):
    global survey_done
    print("Now at waypoint:", msg.seq)
    if msg.seq >= total_wps - 1:   # reached last waypoint
        survey_done = True

# Step 3: main loop
while True:
    if survey_done:
        print("Survey finished ✅")
        if target_lat is not None and target_lon is not None:
            print("Going to detected object...")
            point = LocationGlobalRelative(target_lat, target_lon, 10) # 10m alt
            vehicle.simple_goto(point)
        break
    time.sleep(1)
