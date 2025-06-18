import time
import serial
import struct
from pymavlink import mavutil

# Connect to CubePilot via MAVLink
mav = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

print("Waiting for heartbeat...")
mav.wait_heartbeat()
print(f"Heartbeat received from system {mav.target_system}, component {mav.target_component}")

# Connect TF Mini LiDAR
try:
    ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

def read_lidar():
    """ Reads data from TF Mini LiDAR and returns distance in meters. """
    try:
        while True:
            if ser.read(1) == b'\x59':  # First frame header
                if ser.read(1) == b'\x59':  # Second frame header
                    frame = ser.read(7)  # Read remaining bytes
                    distance = struct.unpack('<H', frame[0:2])[0]  # Distance in cm
                    return distance / 100.0  # Convert to meters
    except Exception as e:
        print(f"Error reading LiDAR: {e}")
        return None  # Return None on failure

# LiDAR Distance Parameters
min_distance = 0.05  # 5 cm
max_distance = 12.0  # 12 meters

while True:
    distance = read_lidar()

    if distance is None:
        continue  # Skip if failed to read

    # Clamp distance within range
    distance = max(min_distance, min(distance, max_distance))

    # Convert to integers for MAVLink
    min_distance_cm = int(min_distance * 100)
    max_distance_cm = int(max_distance * 100)
    current_distance_cm = int(distance * 100)
    # Ensure valid range (MAVLink expects unsigned integers)
    time_boot_ms = int(time.time() * 1000) % 4294967295

    print(f"LiDAR Distance: {distance:.2f} m (Sent: {current_distance_cm} cm)")

    # Send MAVLink distance sensor message
    mav.mav.distance_sensor_send(
        time_boot_ms,         # System uptime (ms) - Unsigned 32-bit
        min_distance_cm,      # Min distance (cm) - Unsigned 16-bit
        max_distance_cm,      # Max distance (cm) - Unsigned 16-bit
        current_distance_cm,  # Current distance (cm) - Unsigned 16-bit
        0,                    # Type (0 = Laser)
        0,                    # ID (default: 0)
        0,                    # Orientation (0 = Forward-facing)
        0,                    # Covariance (0 = Unknown)
        255                   # Signal quality (0-100, or 255 if unknown)
    )

    time.sleep(0.1)  # Send every 100ms