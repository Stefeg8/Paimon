from pymavlink import mavutil

def arm_drone(master):
    master.mav.command_long_send(
        master.target_system,  # System ID
        master.target_component,  # Component ID
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command to arm/disarm
        0,  # Confirmation (0 = no confirmation)
        1,  # Arm the drone (0 = disarm, 1 = arm)
        0, 0, 0, 0, 0, 0  # Other parameters are not needed for this command
    )

if __name__ == "__main__":
    #master = mavutil.mavlink_connection("udp:<drone_ip>:14550") #use this on macos, make sure udp is set to 14550
    master = mavutil.mavlink_connection("COM4", baud=57600)  # use this with telemetry radio on windows. Usage: device manager->ports and then find your port
    # make sure to use the correct COM port on windows
    master.wait_heartbeat()
    arm_drone(master)
