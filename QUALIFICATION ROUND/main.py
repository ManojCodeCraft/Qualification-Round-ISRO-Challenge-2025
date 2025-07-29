import time
from pymavlink import mavutil


def connect_mavlink():
    print("Connecting via MAVLink...")
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
    master.wait_heartbeat()
    print("MAVLink connected!")
    return master


def get_altitude(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
    if msg:
        alt = msg.relative_alt / 1000.0  
        print(f"Altitude : {alt:.2f} m")
        return alt
    else:
        print("No altitude data received!")
        return None


def get_vertical_velocity(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
    if msg:
        vz = msg.vz / 100.0  
        return vz
    else:
        print("No vertical velocity data!")
        return None


def get_optical_flow_xy(master):
    for msg_type in ["OPTICAL_FLOW_RAD", "OPTICAL_FLOW"]:
        msg = master.recv_match(type=msg_type, blocking=True, timeout=2.0)
        if msg:
            try:
                opt_x = msg.integrated_x
                opt_y = msg.integrated_y
            except AttributeError:
                opt_x = msg.flow_x
                opt_y = msg.flow_y
            print(f"Optical Flow => opt_x: {opt_x:.4f}, opt_y: {opt_y:.4f}")
            return opt_x, opt_y
    print("No optical flow data!")
    return None, None


def arm_motors(master):
    print("🛠 Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed")


def set_loiter_mode(master):
    print("Switching to LOITER mode...")
    mode = 'LOITER'
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode}")


def takeoff_to_altitude(master, target_alt):
    print(f"Taking off to {target_alt} meters...")
    throttle = 1500

    while True:
        altitude = get_altitude(master)
        if altitude is None:
            continue

        if altitude < target_alt * 0.95:
            throttle = min(throttle + 10, 1700)
            print(f"Increasing throttle: {throttle}")
        else:
            print(f"Target altitude reached: {altitude:.2f}m")
            break

        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0, 0, throttle, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)


def hover_with_telemetry(master, duration=30):
    print(f"Hovering for {duration} seconds with telemetry logs...")
    hover_time = 0

    while hover_time < duration:
        altitude = get_altitude(master)
        vz = get_vertical_velocity(master)
        opt_x, opt_y = get_optical_flow_xy(master)
        battery_voltage = get_battery_voltage(master)

        if None not in (altitude, vz, opt_x, opt_y, battery_voltage):
            print(f"TELEMETRY => Altitude(Z): {altitude:.2f}m | Vz: {vz:.2f} m/s | opt_x: {opt_x:.4f} | opt_y: {opt_y:.4f} | Battery: {battery_voltage:.2f} V")
        else:
            print("Incomplete telemetry data")

        
        if battery_voltage is not None and battery_voltage < 14.8:
            print("Low battery voltage detected! Initiating emergency landing...")
            gradual_landing(master)
            return

        time.sleep(1)
        hover_time += 1

    
    print("Hovering completed! Initiating gradual landing...")
    gradual_landing(master)


def gradual_landing(master):
    print("Starting gradual landing...")

    while True:
        altitude = get_altitude(master)
        if altitude is None:
            continue

        if altitude <= 0.3:
            print("Touchdown confirmed")
            break

        if altitude > 1.5:
            throttle = 1350
        elif 1.0 < altitude <= 1.5:
            throttle = 1300
        elif 0.5 < altitude <= 1.0:
            throttle = 1250
        else:
            throttle = 1200

        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0, 0, throttle, 0, 0, 0, 0, 0
        )
        print(f"Descending - Throttle: {throttle}")
        time.sleep(1)

   
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 1000, 0, 0, 0, 0, 0
    )

    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Motors disarmed")

def get_battery_voltage(master):
    msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=2.0)
    if msg:
        voltage = msg.voltage_battery / 1000.0  
        print(f"Battery Voltage: {voltage:.2f} V")
        return voltage
    else:
        print("⚠ No battery status received!")
        return None

if __name__ == "__main__":
    master = connect_mavlink()
    TARGET_ALTITUDE = 3.0  

    try:
        arm_motors(master)
        set_loiter_mode(master)
        takeoff_to_altitude(master, TARGET_ALTITUDE)
        hover_with_telemetry(master, duration=30)

    finally:
        print("Disconnecting...")
        master.close()
        print("Disconnected successfully")
