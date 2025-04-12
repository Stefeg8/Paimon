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
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask=0b00001111,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=thrust
        )
        time.sleep(0.05)  # 20 Hz

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
    1,
    0, 0, 0, 0, 0, 0
)
print("drone armed")
time.sleep(3)

# Send a few setpoints before switching to Offboard (PX4 requires it)
print("sending pre-offboard attitude targets")
for _ in range(20):
    master.mav.set_attitude_target_send(
        int((time.time() - start_time) * 1000),
        master.target_system,
        master.target_component,
        type_mask=0b00001111,
        q=[1, 0, 0, 0],
        body_roll_rate=0,
        body_pitch_rate=0,
        body_yaw_rate=0,
        thrust=0.0
    )
    time.sleep(0.05)

# Switch to OFFBOARD mode (custom mode 6 in PX4)
print("switching to OFFBOARD mode")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    6,  # OFFBOARD mode number in PX4
    0, 0, 0, 0, 0
)
time.sleep(1)
print("OFFBOARD mode set")

# Apply thrusts
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
    0,
    0, 0, 0, 0, 0, 0
)
print("disarmed")

print("switching to MANUAL mode")

# Mode 1 = MANUAL in PX4
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    1,  # PX4 manual mode
    0, 0, 0, 0, 0
)
time.sleep(1)
print("mode switched to MANUAL")