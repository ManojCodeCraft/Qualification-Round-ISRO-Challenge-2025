import time
import serial
import struct
from pymavlink import mavutil

mav = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

print("Waiting for heartbeat...")
mav.wait_heartbeat()
print(f"Heartbeat received from system {mav.target_system}, component {mav.target_component}")


try:
    ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

def read_lidar():
    """ Reads data from TF Mini LiDAR and returns distance in meters. """
    try:
        while True:
            if ser.read(1) == b'\x59': 
                if ser.read(1) == b'\x59': 
                    frame = ser.read(7) 
                    distance = struct.unpack('<H', frame[0:2])[0]  
                    return distance / 100.0  
    except Exception as e:
        print(f"Error reading LiDAR: {e}")
        return None  


min_distance = 0.05  
max_distance = 12.0  

while True:
    distance = read_lidar()

    if distance is None:
        continue  

   
    distance = max(min_distance, min(distance, max_distance))

    min_distance_cm = int(min_distance * 100)
    max_distance_cm = int(max_distance * 100)
    current_distance_cm = int(distance * 100)
    
    time_boot_ms = int(time.time() * 1000) % 4294967295

    print(f"LiDAR Distance: {distance:.2f} m (Sent: {current_distance_cm} cm)")

    
    mav.mav.distance_sensor_send(
        time_boot_ms,         
        min_distance_cm,      
        max_distance_cm,      
        current_distance_cm,  
        0,                    
        0,                    
        0,                    
        0,                    
        255                  
    )

    time.sleep(0.1)  
