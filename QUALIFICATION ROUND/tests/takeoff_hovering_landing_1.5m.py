import time
from pymavlink import mavutil


def connect_mavlink():
    print("🔄 Connecting via MAVLink...")
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
    master.wait_heartbeat()
    print("✅ MAVLink connected!")
    return master


def get_lidar_altitude(master):
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=2.0)
    if msg:
        lidar_alt = msg.current_distance / 100.0  # cm to meters
        print(f"📡 LIDAR Altitude: {lidar_alt:.2f} m")
        return lidar_alt
    else:
        print("⚠ No LiDAR data received!")
        return None


def arm_motors(master):
    print("🛠 Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("✅ Motors armed")


def set_alt_hold_mode(master):
    print("🔄 Switching to ALT_HOLD mode...")
    mode = 'ALT_HOLD'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"✅ Mode set to {mode}")


def takeoff_to_altitude(master, target_alt):
    print(f"🚀 Taking off to {target_alt} meters using LIDAR...")

    throttle = 1500  
    while True:
        lidar_alt = get_lidar_altitude(master)
        if lidar_alt is None:
            continue

        
        if lidar_alt < target_alt * 0.95:
            throttle = min(throttle + 10, 1700)
            print(f"⬆ Increasing throttle: {throttle}")
        else:
            print(f"✅ Target altitude reached: {lidar_alt:.2f}m")
            break

        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0, 0, throttle, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)


def hover_using_lidar(master, target_altitude, duration=10):
    print(f"🛸 Hovering at {target_altitude}m for {duration} seconds using LIDAR (Conditional Control)...")

    hover_time = 0
    neutral_throttle = 1500

    while hover_time < duration:
        lidar_alt = get_lidar_altitude(master)
        if lidar_alt is None:
            continue

        throttle = neutral_throttle  

        print(f"✅ Stable at {lidar_alt:.2f}m - Holding throttle")

        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0, 0, throttle, 0, 0, 0, 0, 0
        )

        hover_time += 0.3
        time.sleep(0.3)

    print("✅ Hover complete, resetting throttle")
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, neutral_throttle, 0, 0, 0, 0, 0
    )


def gradual_landing(master):
    print("🛬 Starting gradual landing...")

    while True:
        lidar_alt = get_lidar_altitude(master)
        if lidar_alt is None:
            continue

        if lidar_alt <= 0.3:
            print("✅ Touchdown confirmed")
            break

       
        if lidar_alt > 1.5:
            throttle = 1350
        elif 1.0 < lidar_alt <= 1.5:
            throttle = 1300
        elif 0.5 < lidar_alt <= 1.0:
            throttle = 1250
        else:
            throttle = 1200

        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0, 0, throttle, 0, 0, 0, 0, 0
        )
        print(f"🔽 Descending - Throttle: {throttle}")
        time.sleep(1)

    
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 1000, 0, 0, 0, 0, 0
    )

    
    print("🛑 Disarming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    print("✅ Motors disarmed")


if __name__ == "__main__":
    master = connect_mavlink()
    TARGET_ALTITUDE = 1.5 

    try:
        arm_motors(master)
        set_alt_hold_mode(master)
        takeoff_to_altitude(master, TARGET_ALTITUDE)
        hover_using_lidar(master, TARGET_ALTITUDE, duration=10)  
        gradual_landing(master)

    finally:
        print("🔌 Disconnecting...")
        master.close()
        print("✅ Disconnected successfully!")
