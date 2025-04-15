from pymavlink import mavutil
import time

# Connect to Pixhawk
master = mavutil.mavlink_connection("COM4", baud=57600)
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# Arm the drone
print("Arming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # Arm
    0, 0, 0, 0, 0, 0
)
time.sleep(3)

# Send RC overrides to motors
# Channels 1–4: Typically motor outputs (depending on mixer)
# Values around 1500-1700 us usually spin motors slowly (max ~1900)
print("Sending RC override to spin motors")
for i in range(100):  # Send for ~5 seconds
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1600, 1600, 1600, 1600,  # Channels 1-4
        0, 0, 0, 0               # Channels 5-8 (unused here)
    )
    time.sleep(0.05)

# Stop motors by resetting RC overrides
print("Stopping motors")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0,
    0, 0, 0, 0
)

# Disarm
print("Disarming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,  # Disarm
    0, 0, 0, 0, 0, 0
)

print("Done.")
