from pymavlink import mavutil
import time

# Connect to the Pixhawk flight controller
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
master.wait_heartbeat()
print("Connected to system ID:", master.target_system)

# Set GUIDED mode
def set_guided_mode():
    print("Setting mode to GUIDED...")
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    # Confirm ACK
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print("Mode ACK:", ack)

# Arm the drone
def arm():
    print("Arming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Armed!")

# Send upward velocity to climb (manual takeoff)
def manual_takeoff(climb_rate=0.5, duration=5):
    print(f"Taking off: climbing at {climb_rate} m/s for {duration} seconds...")
    for i in range(duration):
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # Enable vx, vy, vz, yaw_rate
            0, 0, 0,             # x, y, z (not used)
            0, 0, -climb_rate,   # vx, vy, vz (vz negative = up)
            0, 0, 0,
            0, 0
        )
        print(f"Climbing... {i+1}s")
        time.sleep(1)

    # Stop movement
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0,   # stop
        0, 0, 0,
        0, 0
    )
    print("Hovering...")

# Land the drone
def land():
    print("Landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

# Disarm
def disarm():
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Disarmed.")

# === Main Sequence ===
set_guided_mode()
arm()
manual_takeoff(climb_rate=0.5, duration=5)
time.sleep(5)
land()
time.sleep(10)
disarm()
