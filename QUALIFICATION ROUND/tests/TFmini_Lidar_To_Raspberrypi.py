import serial
import time

ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
while True:
    count = ser.in_waiting  
    if count > 8:
        recv = ser.read(9)  
        if recv[0] == 0x59 and recv[1] == 0x59:  
            distance = recv[2] + recv[3] * 256  
            print(f"Distance: {distance} cm")
    time.sleep(0.01)
