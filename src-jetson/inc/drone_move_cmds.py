from pymavlink import mavutil
import time
import math

def pixels_to_degrees(pixel_x, pixel_y):
    # Camera resolution
    resolution_x = 3280
    resolution_y = 2464
    
    # Field of view in degrees
    fov_degrees = 160
    
    # Calculate the angle of view in radians
    fov_radians = math.radians(fov_degrees)
    
    # Calculate the pixel coordinates relative to the center of the image
    center_x = resolution_x / 2
    center_y = resolution_y / 2
    relative_x = pixel_x - center_x
    relative_y = pixel_y - center_y
    
    # Calculate the angle in radians
    angle_x = math.atan2(relative_x, resolution_x / (2 * math.tan(fov_radians / 2)))
    angle_y = math.atan2(relative_y, resolution_y / (2 * math.tan(fov_radians / 2)))
    
    # Convert the angle to degrees
    angle_x_degrees = math.degrees(angle_x)
    angle_y_degrees = math.degrees(angle_y)
    
    return angle_x_degrees, angle_y_degrees

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
