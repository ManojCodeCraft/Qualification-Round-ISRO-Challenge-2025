import time
from pymavlink import mavutil

# ✅ Connect to Cube+ via MAVLink
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

# ✅ Wait for the first heartbeat
print("⏳ Waiting for heartbeat from Cube+...")
master.wait_heartbeat()
print("✅ Cube+ connection established!")

# ✅ Main loop to read barometer & LiDAR altitude
while True:
    # Fetch barometer altitude
    baro_alt = None
    baro_msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if baro_msg:
        baro_alt = baro_msg.relative_alt / 1000.0  # Convert mm to meters

    # Fetch LiDAR altitude
    lidar_alt = None
    lidar_msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    if lidar_msg:
        lidar_alt = lidar_msg.current_distance / 100.0  # Convert cm to meters
        # Display results only if both values are available
    if baro_alt is not None and lidar_alt is not None:
        print(f"Barometer Altitude: {baro_alt:.2f} m | LiDAR Altitude: {lidar_alt:.2f} m")

    # Small delay for stable readings
    time.sleep(0.75)