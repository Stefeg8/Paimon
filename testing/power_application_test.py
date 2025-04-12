from pymavlink import mavutil
import time
import threading

def send_heartbeat(master):
    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(1)

def send_thrust(master, start_time, thrust, duration):
    end_time = time.time() + duration
    while time.time() < end_time:
        master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),  # time_boot_ms
            master.target_system,
            master.target_component,
            type_mask=0b00001111,  # ignore attitude and body rates
            q=[1, 0, 0, 0],  # dummy quaternion, will be ignored
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=thrust
        )
        time.sleep(0.05)  # 20 Hz rate

print("Initializing connection")
master = mavutil.mavlink_connection("COM4", baud=57600)
print("working...")
master.wait_heartbeat()
print(f"connection established with system {master.target_system}")

start_time = time.time()

# Start sending heartbeats in the background
heartbeat_thread = threading.Thread(target=send_heartbeat, args=(master,), daemon=True)
heartbeat_thread.start()

# Arm the drone
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # Arm
    0, 0, 0, 0, 0, 0
)
print("drone armed")
time.sleep(3)

# Apply different thrust levels with duration
print("applying 0.25 thrust")
send_thrust(master, start_time, thrust=0.25, duration=3)

print("applying 0.5 thrust")
send_thrust(master, start_time, thrust=0.5, duration=3)

print("applying 0.75 thrust")
send_thrust(master, start_time, thrust=0.75, duration=3)

print("applying 0 thrust and initiating shutdown")
send_thrust(master, start_time, thrust=0.0, duration=1)

# Disarm the drone
print("disarming")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,  # Disarm
    0, 0, 0, 0, 0, 0
)
print("disarmed")
