from dronekit import connect,VehicleMode,LocationGlobalRelative
#import dronekit_sitl
import time
import argparse

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
#we can also add multiple parser using the same add_argument thing in the above!
args = parser.parse_args()
connection_string = args.connect
print("Connection to the vehicle on ",connection_string)
vehicle = connect(connection_string,wait_ready=True)

def arm_and_takeoff(altitude):
    print("Arming motors")
    while not vehicle.is_armable:
        print("Not in an armable state yet!")
        time.sleep(1)
    vehicle.mode = VehicleMode('GUIDED')
    vehicle.armed = True
    while not vehicle.armed:
        print("Not Armed yet!")
        time.sleep(1)
    print("Armed.... & ready to TAKEOFF")
    vehicle.simple_takeoff(altitude)
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(current_alt)
        if current_alt >= altitude:
            print("Reached the determined location!")
            break
        time.sleep(1)

arm_and_takeoff(20)

vehicle.airspeed = 5

print("GOing to WP1!")

wpl = LocationGlobalRelative(12.9719, 80.0429, 10)
vehicle.simple_goto(wpl)

time.sleep(30)

vehicle.mode = VehicleMode("RTL")
vehicle.close()