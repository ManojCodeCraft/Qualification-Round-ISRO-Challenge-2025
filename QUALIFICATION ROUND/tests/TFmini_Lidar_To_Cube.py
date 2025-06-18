import time
from pymavlink import mavutil

# ✅ Connect to Cube+ via MAVLink (Check the correct port)
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

# ✅ Wait for the first heartbeat (Ensures connection is active)
print("⏳ Waiting for heartbeat from Cube+...")
master.wait_heartbeat()
print("✅ Cube+ connection established!")

# ✅ Fetch LiDAR distance in real-time
while True:
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=1)
    if msg:
        distance = msg.current_distance / 100.0  # Convert cm to meters
        print(f"📡 LiDAR Distance: {distance:.2f} meters")
    else:
        print("⚠️ No LiDAR data received! Retrying...")

    time.sleep(0.1)  # Small delay for stable readings