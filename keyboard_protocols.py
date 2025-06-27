from pymavlink import mavutil
import time

# Connect to Pixhawk via serial or UDP
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
master.wait_heartbeat()
print("Connected to drone!")

def arm_and_takeoff(target_altitude=2):
    master.set_mode("GUIDED")
    print("Arming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Motors armed.")

    print("Taking off to", target_altitude, "meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude
    )
    time.sleep(5)

def land():
    print("Landing...")
    master.set_mode("LAND")

def disarm():
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
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
        0b0000111111000111,  # Only velocity + yaw enabled
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate
    )

def stop():
    send_velocity(0, 0, 0, 0)

def menu():
    print("""
========== Drone Keyboard CLI ==========
t : Takeoff
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
    cmd = input("Enter command: ").lower()

    if cmd == 't':
        arm_and_takeoff(2)

    elif cmd == 'l':
        land()

    elif cmd == 'x':
        disarm()
        print("Exiting...")
        break

    elif cmd == 'w':
        send_velocity(0.5, 0, 0)

    elif cmd == 's':
        send_velocity(-0.5, 0, 0)

    elif cmd == 'a':
        send_velocity(0, -0.5, 0)

    elif cmd == 'd':
        send_velocity(0, 0.5, 0)

    elif cmd == 'q':
        send_velocity(0, 0, 0, yaw_rate=-30)

    elif cmd == 'e':
        send_velocity(0, 0, 0, yaw_rate=30)

    elif cmd == 'space':
        stop()

    else:
        print("Unknown command.")
