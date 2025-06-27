from pymavlink import mavutil
import time

# Connect to the Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)  # Change port if needed
master.wait_heartbeat()
print("Heartbeat received. Connected to drone!")

def set_guided_mode():
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()['GUIDED']
    )
    time.sleep(1)

def arm_and_takeoff(target_altitude=2):
    set_guided_mode()

    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed.")

    print(f"Taking off to {target_altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            altitude = msg.relative_alt / 1000.0  # mm to m
            print(f"Altitude: {altitude:.2f} m")
            if altitude >= target_altitude * 0.95:
                print("Reached target altitude.")
                break
        time.sleep(1)

def land():
    print("Landing...")
    master.set_mode(master.mode_mapping()['LAND'])

def disarm():
    print("Disarming motors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors disarmed.")

def send_velocity(vx, vy, vz, yaw_rate=0):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # Only velocity and yaw rate enabled
        0, 0, 0,             # x, y, z positions (unused)
        vx, vy, vz,          # velocity in m/s
        0, 0, 0,             # acceleration (not used)
        0, yaw_rate
    )

def stop():
    send_velocity(0, 0, 0, 0)

def menu():
    print("""
========== Drone Keyboard CLI ==========
t : Arm and Takeoff
l : Land
x : Disarm + Exit
w : Forward
s : Backward
a : Left
d : Right
q : Yaw Left
e : Yaw Right
space : Stop
========================================
""")

# Main control loop
menu()
while True:
    cmd = input("Enter command: ").lower().strip()

    if cmd == 't':
        arm_and_takeoff(2)

    elif cmd == 'l':
        land()

    elif cmd == 'x':
        disarm()
        print("Exiting...")
        break

    elif cmd == 'w':
        send_velocity(0.5, 0, 0)  # Forward

    elif cmd == 's':
        send_velocity(-0.5, 0, 0)  # Backward

    elif cmd == 'a':
        send_velocity(0, -0.5, 0)  # Left

    elif cmd == 'd':
        send_velocity(0, 0.5, 0)   # Right

    elif cmd == 'q':
        send_velocity(0, 0, 0, yaw_rate=-30)  # Yaw left

    elif cmd == 'e':
        send_velocity(0, 0, 0, yaw_rate=30)   # Yaw right

    elif cmd == 'space':
        stop()

    else:
        print("Unknown command. Type again.")
