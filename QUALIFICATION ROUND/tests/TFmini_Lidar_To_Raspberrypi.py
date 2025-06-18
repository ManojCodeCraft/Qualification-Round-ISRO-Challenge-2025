import serial
import time
# Open serial connection for TFmini-S
ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
while True:
    count = ser.in_waiting  # Check if data is available
    if count > 8:
        recv = ser.read(9)  # Read 9 bytes
        if recv[0] == 0x59 and recv[1] == 0x59:  # Check header bytes
            distance = recv[2] + recv[3] * 256  # Calculate distance
            print(f"Distance: {distance} cm")
    time.sleep(0.01)