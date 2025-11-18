from pymavlink import mavutil
import time

# ----------------------------------
# USER CONFIG
# ----------------------------------
connection_string = "COM22"     # Change if needed
baud_rate = 57600

SERVO_CHANNEL = 10              # AUX2 = channel 10
PWM_DEFAULT = 1100              # Starting position
PWM_TRIGGER = 1900              # Trigger position
TRIGGER_DELAY = 3.0             # Seconds
# ----------------------------------

print("Connecting to drone...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# Wait for heartbeat
master.wait_heartbeat()
print("Connected. Heartbeat received.")

# ----------------------------------
# FUNCTION TO SET SERVO POSITION
# ----------------------------------
def set_servo_pwm(channel, pwm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,      # servo number (1–16)
        pwm,          # PWM value
        0, 0, 0, 0, 0
    )
    print(f"Servo {channel} → PWM {pwm}")

# ----------------------------------
# MAIN SEQUENCE
# ----------------------------------

print("\nSetting servo to DEFAULT position...")
set_servo_pwm(SERVO_CHANNEL, PWM_DEFAULT)
time.sleep(1)

print("\nTriggering servo...")
set_servo_pwm(SERVO_CHANNEL, PWM_TRIGGER)
time.sleep(TRIGGER_DELAY)

print("\nReturning servo to DEFAULT position...")
set_servo_pwm(SERVO_CHANNEL, PWM_DEFAULT)

print("\n✔ Servo trigger sequence complete.")
