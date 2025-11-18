from pymavlink import mavutil
MAVLINK_CONN="udp:127.0.0.1:14552"

print("🔌 Connecting to vehicle...")
vehicle = mavutil.mavlink_connection(MAVLINK_CONN)  
vehicle.wait_heartbeat()
print("✅ Connected to system (System ID:", vehicle.target_system, ")")