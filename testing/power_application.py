from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:<drone_ip>:14550") #should be port 14550 but check. PLEASE CHANGE THIS
master.wait_heartbeat()

master.mav.command_long_send(
    master.target_system,  # System ID
    master.target_component,  # Component ID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command to arm/disarm
    0,  # Confirmation (0 = no confirmation)
    1,  # Arm the drone (0 = disarm, 1 = arm)
    0, 0, 0, 0, 0, 0  # Other parameters are not needed for this command
)

type_mask = 0b00001111  # = 15

master.mav.set_attitude_target_send(
    int(master.time_since_boot() * 1000),  # time_boot_ms
    master.target_system,
    master.target_component,
    type_mask=0b00001111,  # ignore attitude and body rates
    q=[1, 0, 0, 0],  # dummy quaternion, will be ignored
    body_roll_rate=0,
    body_pitch_rate=0,
    body_yaw_rate=0,
    thrust=0.5 
)