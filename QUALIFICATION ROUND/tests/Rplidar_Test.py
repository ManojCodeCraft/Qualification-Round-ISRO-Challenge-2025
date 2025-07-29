import sys
from rplidar import RPLidar

import glob
PORTS = glob.glob("/dev/ttyUSB1")
if not PORTS:
    raise Exception(" No LIDAR device detected!")
PORT_NAME = PORTS[0]

lidar = RPLidar(PORT_NAME, baudrate=115200)

def get_lidar_data():
    try:
        print("Starting RPLidar A1 scanning... (Press Ctrl+C to stop)")
        distances = {'left': 999, 'right': 999, 'forward': 999, 'backward': 999}

        while True:
            for scan in lidar.iter_scans():
                for _, angle, distance in scan:
                    if distance > 0:
                        if (315 <= angle <= 359) or (0 <= angle <= 45):  
                            distances['forward'] = min(distances['forward'], distance / 1000.0)
                        elif 135 <= angle <= 225:
                            distances['backward'] = min(distances['backward'], distance / 1000.0)
                        elif 45 <= angle <= 135:  
                            distances['right'] = min(distances['right'], distance / 1000.0)
                        elif 225 <= angle <= 315: 
                            distances['left'] = min(distances['left'], distance / 1000.0)

                print(f"LiDAR Readings - Left: {distances['left']}m, Right: {distances['right']}m, "
                      f"Forward: {distances['forward']}m, Backward: {distances['backward']}m")

    except KeyboardInterrupt:
        print("\nStopping scan by user...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Disconnecting LIDAR...")
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    get_lidar_data()
