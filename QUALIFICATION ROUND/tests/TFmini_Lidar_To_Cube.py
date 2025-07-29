import time
from pymavlink import mavutil


master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

print("Waiting for heartbeat from Cube+...")
master.wait_heartbeat()
print("Cube+ connection established!")

while True:
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=1)
    if msg:
        distance = msg.current_distance / 100.0  
        print(f"LiDAR Distance: {distance:.2f} meters")
    else:
        print("No LiDAR data received! Retrying...")

    time.sleep(0.1)  
