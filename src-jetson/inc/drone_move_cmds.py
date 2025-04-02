from pymavlink import mavutil
import time
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

# Note: a reverse quaternion will effectively reverse the previous rotational command
# We can use this to backtrack, gain distance, etc etc

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles (in degrees) to a quaternion [x, y, z, w].
    """
    roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])  # Convert degrees to radians
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return [qx, qy, qz, qw]

def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion has zero magnitude!")
    return [i / norm for i in q]

def slerp(q1, q2, t):
    q1 = R.from_quat(q1)
    q2 = R.from_quat(q2)
    q_interp = R.slerp(t, [q1, q2]).as_quat()
    return q_interp

def quaternion_to_euler(q):
    """
    Converts a quaternion to Euler angles (roll, pitch, yaw).
    
    :param q: Quaternion as [x, y, z, w]
    :return: Roll, Pitch, Yaw in radians
    """
    x, y, z, w = q

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def set_drone_attitude_smooth(master, current_roll, current_pitch, current_yaw, target_roll, target_pitch, target_yaw, duration=1.0, steps=10, thrust=0.5):
    """
    Smoothly adjusts the drone's roll, pitch, and yaw using SLERP over a set duration.

    :param master: MAVLink connection object
    :param current_roll: Current roll angle in degrees
    :param current_pitch: Current pitch angle in degrees
    :param current_yaw: Current yaw angle in degrees
    :param target_roll: Desired roll angle in degrees
    :param target_pitch: Desired pitch angle in degrees
    :param target_yaw: Desired yaw angle in degrees
    :param duration: Time (in seconds) over which the transition happens
    :param steps: Number of intermediate steps in the transition
    :param thrust: Thrust value (0-1 range, 0.5 = hover)
    """
    # Convert current and target angles to quaternions
    q_start = R.from_euler('xyz', [current_roll, current_pitch, current_yaw], degrees=True).as_quat()
    q_end = R.from_euler('xyz', [target_roll, target_pitch, target_yaw], degrees=True).as_quat()

    # Interpolate the quaternions using SLERP
    for i in range(steps + 1):
        t = i / steps  # Interpolation factor (0 to 1)
        q_interp = slerp(q_start, q_end, t)

        # Send attitude command using interpolated quaternion
        master.mav.set_attitude_target_send(
            int(time.time() * 1e6),
            master.target_system,
            master.target_component,
            0b00000000,  # Ignore body rates, use quaternion
            q_interp,  
            0, 0, 0,  # No roll/pitch/yaw rates
            thrust  # Thrust (0-1)
        )
        
        time.sleep(duration / steps)  # Delay for smooth transition

# Function to calculate attitude adjustments based on person's pixel location
def calculate_attitude_adjustment(person_x, person_y, image_width, image_height, max_angle=30):
    """
    Calculate the required roll, pitch, and yaw to follow a detected person.

    :param person_x: Person's x-coordinate in pixels.
    :param person_y: Person's y-coordinate in pixels.
    :param image_width: Width of the camera image in pixels.
    :param image_height: Height of the camera image in pixels.
    :param max_angle: Maximum angle to rotate in degrees.
    
    :return: (roll, pitch, yaw) adjustments in degrees.
    """
    # Calculate offsets from the center of the image
    offset_x = person_x - (image_width / 2)  # Horizontal offset
    offset_y = person_y - (image_height / 2)  # Vertical offset

    # Normalize offsets to the range [-1, 1]
    norm_offset_x = offset_x / (image_width / 2)
    norm_offset_y = offset_y / (image_height / 2)

    # Calculate pitch and yaw adjustments based on offsets
    pitch = max(-max_angle, min(max_angle, norm_offset_y * max_angle))
    yaw = max(-max_angle, min(max_angle, norm_offset_x * max_angle))

    # Roll may not be needed if the camera is already aligned, so we keep it at 0
    roll = 0

    return roll, pitch, yaw

def get_current_attitude(master):
    """
    Get the current attitude (roll, pitch, yaw) from the drone.

    :param master: MAVLink connection object
    :return: roll, pitch, yaw (in radians)
    """
    # Request the current attitude
    master.mav.request_data_stream_send(
        master.target_system, 
        master.target_component, 
        mavutil.mavlink.MAV_DATA_STREAM_ALL,  # Stream all data
        1,  # Frequency in Hz (1 Hz in this case)
        1   # Enable the data stream
    )

    # Wait for the ATTITUDE message (this might take some time)
    message = master.recv_match(type='ATTITUDE', blocking=True)  # Block until we get the message

    if message:
        # The attitude message contains roll, pitch, yaw (in radians)
        roll = message.roll
        pitch = message.pitch
        yaw = message.yaw
        return roll, pitch, yaw
    else:
        print("Failed to get attitude")
        return None, None, None

def follow_person(master, person_x, person_y, image_width=640, image_height=480, duration=2.0, steps=20, thrust=0.5):
    """
    Follow a person based on their detected position (x, y) in the image frame.
    
    :param master: MAVLink connection object
    :param person_x: x-coordinate of the person in the image.
    :param person_y: y-coordinate of the person in the image.
    :param image_width: Camera image width in pixels.
    :param image_height: Camera image height in pixels.
    :param duration: Time for smooth transition.
    :param steps: Number of steps for SLERP interpolation.
    :param thrust: Thrust level for hovering.
    """
     # Get the current attitude from the drone
    current_roll, current_pitch, current_yaw = get_current_attitude(master)
    
    if current_roll is None or current_pitch is None or current_yaw is None:
        print("Could not get current attitude")
        return

    print(f"Current Attitude - Roll: {current_roll}, Pitch: {current_pitch}, Yaw: {current_yaw}")
    roll, pitch, yaw = calculate_attitude_adjustment(person_x, person_y, image_width, image_height)
    # Use SLERP to smoothly move the drone
    set_drone_attitude_smooth(master, current_roll, current_pitch, current_yaw, roll, pitch, yaw, duration, steps, thrust)

def set_drone_pitch(master, pitch_angle=None, pitch_rate=None, thrust=0.5):
    """
    Sends a MAVLink SET_ATTITUDE_TARGET message to pitch the drone up or down.
    
    :param master: MAVLink connection object
    :param pitch_angle: Target pitch angle in degrees (positive = nose up, negative = nose down). If None, pitch_rate is used.
    :param pitch_rate: Continuous pitch rate in rad/s (negative = pitch down, positive = pitch up). If None, pitch_angle is used.
    :param thrust: Thrust value (0-1 range, 0.5 = hover)
    """
    if pitch_angle is not None:
        # Convert pitch angle to radians and compute quaternion
        pitch_rad = math.radians(pitch_angle)
        q = [0, math.sin(pitch_rad / 2), 0, math.cos(pitch_rad / 2)]
        
        # Send SET_ATTITUDE_TARGET with fixed pitch angle
        master.mav.set_attitude_target_send(
            int(time.time() * 1e6),  # Timestamp in microseconds
            master.target_system,  # Target system ID
            master.target_component,  # Target component ID
            0b00000000,  # Ignore body rates
            q,  # Attitude quaternion
            0,  # Roll rate
            0,  # Pitch rate
            0,  # Yaw rate
            thrust  # Thrust (0-1)
        )
    
    elif pitch_rate is not None:
        # Send SET_ATTITUDE_TARGET with continuous pitch rate
        master.mav.set_attitude_target_send(
            int(time.time() * 1e6),  # Timestamp in microseconds
            master.target_system,  # Target system ID
            master.target_component,  # Target component ID
            0b00000100,  # Ignore attitude quaternion, use body rates
            [0, 0, 0, 1],  # Identity quaternion (no change)
            0,  # Roll rate
            pitch_rate,  # Pitch rate (rad/s)
            0,  # Yaw rate
            thrust  # Thrust (0-1)
        )

def hold_pitch_angle(master, pitch_angle, thrust=0.5, duration=5):
    """
    Continuously commands the drone to maintain a pitch angle.

    :param master: MAVLink connection object
    :param pitch_angle: Target pitch angle in degrees (positive = nose up, negative = nose down).
    :param thrust: Thrust value (0-1 range, 0.5 = hover)
    :param duration: Duration to hold the attitude (in seconds)
    """

    pitch_rad = math.radians(pitch_angle)
    q = [0, math.sin(pitch_rad / 2), 0, math.cos(pitch_rad / 2)]

    start_time = time.time()
    while time.time() - start_time < duration:
        master.mav.set_attitude_target_send(
            int(time.time() * 1e6),
            master.target_system,
            master.target_component,
            0b00000000,  # Ignore body rates
            q,  # Attitude quaternion
            0,  # Roll rate
            0,  # Pitch rate
            0,  # Yaw rate
            thrust  # Thrust (0-1)
        )
        time.sleep(0.02)  # Send at 50Hz (every 20ms)

def set_yaw_angle(master, yaw_angle, thrust=0.5, duration=2):
    import time
    import math

    yaw_rad = math.radians(yaw_angle)
    q = [0, 0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)]

    start_time = time.time()
    while time.time() - start_time < duration:
        master.mav.set_attitude_target_send(
            int(time.time() * 1e6),
            master.target_system,
            master.target_component,
            0b00000000,  # Ignore body rates
            q,  # Attitude quaternion
            0,  # Roll rate
            0,  # Pitch rate
            0,  # Yaw rate
            thrust  # Thrust (0-1)
        )
        time.sleep(0.02)  # Send at 50Hz

def set_attitude(master, pitch, yaw, thrust=0.5, duration=2):
    """
    Commands the drone to a specific pitch and yaw angle.

    :param master: MAVLink connection object
    :param pitch: Pitch angle in degrees (positive = nose up, negative = nose down).
    :param yaw: Yaw angle in degrees (positive = right, negative = left).
    :param thrust: Thrust level (0.0 - 1.0).
    :param duration: Time to hold this attitude.
    """
    q = euler_to_quaternion(0, pitch, yaw)

    start_time = time.time()
    while time.time() - start_time < duration:
        master.mav.set_attitude_target_send(
            int(time.time() * 1e6),
            master.target_system,
            master.target_component,
            0b00000000,  # Ignore body rates
            q,  # Quaternion
            0,  # Roll rate
            0,  # Pitch rate
            0,  # Yaw rate
            thrust  # Thrust (0-1)
        )
        time.sleep(0.02)  # 50Hz update rate
