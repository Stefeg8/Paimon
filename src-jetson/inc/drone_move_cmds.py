from pymavlink import mavutil
import time
import math
from scipy.spatial.transform import Rotation as R
import numpy as np

# Note: a reverse quaternion will effectively reverse the previous rotational command
# We can use this to backtrack, gain distance, etc etc

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles (in degrees) to a quaternion [w, x, y, z] for MAVLink.
    """
    roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])
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

    return [qw, qx, qy, qz]

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
    
def get_current_attitude_quaternion(master):
    master.wait_heartbeat()
    while True:
        msg = master.recv_match(type='ATTITUDE_QUATERNION', blocking=True)
        if msg:
            q = [msg.q1, msg.q2, msg.q3, msg.q4]
            return q
        
def angles_to_direction_vector(delta_x_deg, delta_y_deg):
    # Convert to radians
    yaw_rad = np.radians(delta_x_deg)
    pitch_rad = np.radians(delta_y_deg)

    # In the camera frame:
    # - Forward is +Z
    # - Right is +X
    # - Down is +Y
    x = np.tan(yaw_rad)      # left/right offset
    y = np.tan(pitch_rad)    # up/down offset
    z = 1.0                  # forward

    # Direction vector (not yet normalized)
    vec = np.array([x, y, z])
    return vec / np.linalg.norm(vec)
'''deprecated
def angles_to_direction_vector(delta_yaw_deg, delta_pitch_deg):
    yaw_rad = np.radians(delta_yaw_deg)
    pitch_rad = np.radians(delta_pitch_deg)
    
    # Assume forward is +Z in camera frame, X is right, Y is down
    x = np.sin(yaw_rad)
    y = np.sin(pitch_rad)
    z = np.cos(yaw_rad) * np.cos(pitch_rad)
    
    vec = np.array([x, y, z])
    return vec / np.linalg.norm(vec)
'''
def camera_to_world_vector(direction_cam, q_current, cam_to_drone=None):
    r_current = R.from_quat(q_current)
    
    # If camera is mounted at an angle, account for that
    if cam_to_drone:
        r_cam_to_drone = R.from_quat(cam_to_drone)
        direction_drone = r_cam_to_drone.apply(direction_cam)
    else:
        direction_drone = direction_cam
    
    # Transform to world frame
    direction_world = r_current.apply(direction_drone)
    return direction_world

def look_rotation(direction_world, up=np.array([0, 0, 1])):
    z = direction_world / np.linalg.norm(direction_world)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:  # up and direction are nearly aligned
        x = np.array([1, 0, 0])  # fallback
    x /= np.linalg.norm(x)
    y = np.cross(z, x)

    rot_matrix = np.vstack([x, y, z]).T
    return R.from_matrix(rot_matrix).as_quat()

def slerp_rotation(q_current, q_target, t):
    r_current = R.from_quat(q_current)
    r_target = R.from_quat(q_target)
    slerp = R.slerp(0, 1, [r_current, r_target])
    r_interp = slerp(t)
    return r_interp.as_quat()

def pitch_up_calc(quat, pitch_deg):
    """
    Adjusts the pitch of a given quaternion by a small angle.

    Parameters:
    - quat: array-like, shape (4,) in [x, y, z, w] format
    - pitch_deg: float, the amount of pitch to add in degrees

    Returns:
    - new_quat: array, shape (4,), the adjusted quaternion [x, y, z, w]
    """
    # Original orientation as a rotation object
    original_rot = R.from_quat(quat)

    # Pitch-up rotation about the y-axis
    pitch_rot = R.from_euler('y', pitch_deg, degrees=True)

    # Compose rotations: apply pitch on top of original orientation
    new_rot = pitch_rot * original_rot

    return new_rot.as_quat()

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

def set_attitude1(master, pitch, yaw, thrust=0.5, duration=2):
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

def set_attitude(master, q, thrust=0.5, duration=2.0):
    """
    Set drone's attitude using a target quaternion (w, x, y, z).

    :param master: MAVLink connection
    :param q: Quaternion [w, x, y, z]
    :param thrust: Thrust (0.0 - 1.0)
    :param duration: Duration to maintain this attitude
    """
    if len(q) != 4:
        raise ValueError("Quaternion must have 4 elements: [w, x, y, z]")

    type_mask = 0b00000111  # Ignore roll, pitch, yaw rates

    start_time = time.time()
    while time.time() - start_time < duration:
        master.mav.set_attitude_target_send(
            int(time.time() * 1e6),           # time_boot_ms
            master.target_system,
            master.target_component,
            type_mask,
            q,                                # Quaternion (w, x, y, z)
            0, 0, 0,                          # Roll/Pitch/Yaw rate (ignored)
            thrust
        )
        time.sleep(0.02)



def get_pitch_increment(distance):
    # Distance thresholds
    SAFE_DISTANCE = 220
    CAUTION_DISTANCE = 180
    CRITICAL_DISTANCE = 100
    if distance > SAFE_DISTANCE:
        return 0
    elif distance > CAUTION_DISTANCE:
        return 5  # gentle pitch-up
    elif distance > CRITICAL_DISTANCE:
        return 10  # more aggressive pitch-up
    else:
        return 0  # too close, hover
    
def initialize_offboard(master, start_time=time.time()):
    print("sending pre-offboard attitude targets") # have to do this first or else it doesn't work
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