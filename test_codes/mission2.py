import json
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

vehicle = connect('port', wait_ready=True)

def arm_and_takeoff(altitude):
    print("Arming motors")
    while not vehicle.is_armable:
        print("Waiting for vehicle to be armable...")
        time.sleep(1)
    vehicle.mode = VehicleMode('GUIDED')
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(altitude)
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_alt}")
        if current_alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

file_name = 'tags.json'

def writeup(stats):
    with open(file_name, 'w') as file:
        json.dump(stats, file, indent=4)


#######################################################################


arm_and_takeoff(20)

waypoints = [
    (12.9719, 80.0429, 10),
    (12.9719, 80.0429, 10),
    (12.9719, 80.0429, 10)  
]                                   ###VALUES NOT YET CHANGED!!!!###

collect = {}
count = 1

for lat, lon, alt in waypoints:
    print(f"Going to WP{count}!")
    vehicle.airspeed = 5
    target_location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target_location)
    time.sleep(10)

    current_location = vehicle.location.global_frame
    current_alt = vehicle.location.global_relative_frame.alt

    collect[f"Waypoint {count}"] = {
        "Altitude": round(current_alt, 2),
        "Latitude": round(current_location.lat, 6),
        "Longitude": round(current_location.lon, 6),
        "Speed": vehicle.airspeed
    }
    count += 1

writeup(collect)

vehicle.mode = VehicleMode("RTL")
vehicle.close()
