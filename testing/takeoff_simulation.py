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
            type_mask=0b10000000,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=thrust
        )
        time.sleep(0.05)  # 20 Hz

def simulate_takeoff_and_landing(master, start_time, takeoff_thrust=0.6, ramp_duration=3, hold_duration=2):
    steps = int(ramp_duration / 0.05) 
    thrust_values_up = [i * (takeoff_thrust / steps) for i in range(steps + 1)]
    thrust_values_down = list(reversed(thrust_values_up))

    # Ramp up
    print("ramping up")
    for thrust in thrust_values_up:
        master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask=0b10000000,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=thrust
        )
        time.sleep(0.05)

    # Hover
    print("hovering")
    hover_end = time.time() + hold_duration
    while time.time() < hover_end:
        master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask=0b10000000,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=0.51
        )
        time.sleep(0.05)

    # Ramp down
    print("ramping down thrust")
    for thrust in thrust_values_down:
        master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask=0b10000000,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=0.46
        )
        time.sleep(0.05)

    print("hopefully touchdown")

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
        type_mask=0b10000000,
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

# Takeoff simulation
# Pass takeoff_thrust as the 3rd argument to get a custom thrust value. Default is 0.6 in the function
takeoff_thrust = 0.52 # Ramp up slowly from 0.5 to 0.65 in your testing. You can adjust this value as needed. do 0.02 increments ig
ramp_duration = 10 # Adjust this value as needed. its how long it takes for the drone to go up from 0 thrust to takeoff thrust. don't really need to change this though
simulate_takeoff_and_landing(master, start_time, takeoff_thrust, ramp_duration)
#simulate_takeoff_and_landing(master, start_time) # this is the default function. you can use this if you want. 

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

print("switching back to manual mode")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    1,
    0, 0, 0, 0, 0
)
print("Manual mode activated")