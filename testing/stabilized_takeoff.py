from pymavlink import mavutil
import time
import threading

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ]
    return q

def get_local_position(master):
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    return msg.x, msg.y, msg.z  # z is negative upward (NED frame)

def get_attitude(master):
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    return msg.roll, msg.pitch, msg.yaw  # In radians

def get_altitude(master):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg.relative_alt / 1000.0  # in meters

def stabilize_and_hover(master, start_time, hold_duration, 
                        target_roll=0.0, target_pitch=0.0, 
                        target_yaw=0.0, target_altitude=1.0):
    print("starting stabilization hover...")

    pid_roll = PID(kp=3.0, ki=0.0, kd=0.5)
    pid_pitch = PID(kp=3.0, ki=0.0, kd=0.5)
    pid_yaw = PID(kp=1.0, ki=0.0, kd=0.1)
    pid_altitude = PID(kp=1.0, ki=0.2, kd=0.3)

    last_time = time.time()
    end_time = last_time + hold_duration

    while time.time() < end_time:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Read sensors
        roll, pitch, yaw = get_attitude(master)
        altitude = get_altitude(master)

        # Compute error
        error_roll = target_roll - roll
        error_pitch = target_pitch - pitch
        error_yaw = target_yaw - yaw
        error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi  # wrap [-π, π]
        error_altitude = target_altitude - altitude

        # PID control
        roll_correction = pid_roll.compute(error_roll, dt)
        pitch_correction = pid_pitch.compute(error_pitch, dt)
        yaw_rate = pid_yaw.compute(error_yaw, dt)
        thrust = 0.5 + pid_altitude.compute(error_altitude, dt)

        # Clamp corrections
        roll_correction = max(min(roll_correction, 0.087), -0.087)
        pitch_correction = max(min(pitch_correction, 0.087), -0.087)
        yaw_rate = max(min(yaw_rate, 1.0), -1.0)
        thrust = max(min(thrust, 0.7), 0.4)  # prevent overthrust or cutoffs

        # Quaternion from corrected roll/pitch and fixed yaw
        q = euler_to_quaternion(roll_correction, pitch_correction, target_yaw)

        # Send attitude + yaw rate + thrust
        master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask=0b00000100,  # yaw rate enabled
            q=q,
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=yaw_rate,
            thrust=thrust
        )

        time.sleep(0.05)


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

    
    stabilize_and_hover(master, start_time, hold_duration)

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
            thrust=0.49
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
takeoff_thrust = 0.54 # Ramp up slowly from 0.5 to 0.65 in your testing. You can adjust this value as needed. do 0.02 increments ig
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