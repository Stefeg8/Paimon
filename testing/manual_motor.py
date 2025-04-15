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

def send_actuator_controls(master, duration=5):
    print("Sending actuator commands")
    end_time = time.time() + duration
    while time.time() < end_time:
        # Channel mapping (depends on your PX4 mixer):
        # usually channels 0–3 for quadrotor = motors
        # values between -1 and 1 (0 = stop, 1 = full throttle)
        master.mav.set_actuator_control_target_send(
            0,  # Timestamp (ignored)
            master.target_system,
            master.target_component,
            0,  # Actuator group 0 (for main outputs)
            [0.5, 0.5, 0.5, 0.5, 0, 0, 0, 0]  # 4 motors, half throttle
        )
        time.sleep(0.05)

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection("COM4", baud=57600)
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system} component {master.target_component}")

# Start heartbeats
heartbeat_thread = threading.Thread(target=send_heartbeat, args=(master,), daemon=True)
heartbeat_thread.start()
# Arm
print("Arming motors")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # 1 = arm
    0, 0, 0, 0, 0, 0
)
time.sleep(2)

# Pre-send setpoints before entering OFFBOARD
print("Sending pre-offboard actuator setpoints")
for _ in range(10):
    master.mav.set_actuator_control_target_send(
        0,
        master.target_system,
        master.target_component,
        0,
        [0.0] * 8
    )
    time.sleep(0.1)

# Set mode to OFFBOARD
print("Switching to OFFBOARD mode")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    6,  # PX4 OFFBOARD mode
    0, 0, 0, 0, 0
)
time.sleep(1)

# Spin all motors
send_actuator_controls(master, duration=5)

# Disarm
print("Disarming motors")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,  # disarm
    0, 0, 0, 0, 0, 0
)

print("All done.")
