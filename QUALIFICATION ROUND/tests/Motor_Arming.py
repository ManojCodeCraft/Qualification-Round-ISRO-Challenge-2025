from pymavlink import mavutil
import time

# Connect to the Cube+
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
master.wait_heartbeat()
print("✅ Heartbeat received!")

# Arm the motors
print("Arming motors...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

# Wait for 10 seconds while motors are armed
time.sleep(10)

# Disarm the motors
print("Disarming motors...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0
)

print("✅ Disarmed successfully!")