from dronekit import connect, VehicleMode
import time

# --- Serial connection settings ---
connection_port = 'udp:127.0.0.1:14552'    # Your Bluetooth COM port
baud_rate = 115200           # Typical baud rate for flight controllers or telemetry links

print(f"Attempting to connect to the vehicle on {connection_port} at {baud_rate} baud...")

try:
    # Establish connection to the vehicle
    # wait_ready=True ensures parameters are downloaded before proceeding
    vehicle = connect(connection_port, baud=baud_rate, wait_ready=True, timeout=60)
    print("✅ Connection established successfully!")

    # --- Display basic vehicle information ---
    print(f" Autopilot Firmware Version: {vehicle.version}")
    print(f" Global Location (latitude, longitude, altitude): {vehicle.location.global_frame}")
    print(f" Relative Altitude: {vehicle.location.global_relative_frame.alt}")
    print(f" Current Flight Mode: {vehicle.mode.name}")
    print(f" Armed State: {vehicle.armed}")

except Exception as e:
    print("❌ Unable to connect to the vehicle:")
    print(e)

finally:
    # Safely close the connection when done
    try:
        vehicle.close()
    except Exception:
        pass

    print("🔚 Connection test completed.")
