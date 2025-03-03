import drone_move_cmds as dmc
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("udp:127.0.0.1:14550") #should be port 14550 but check 
master.wait_heartbeat()


# Example Usage:
dmc.hold_pitch_angle(master, pitch_angle=15, duration=5)
    
dmc.set_drone_pitch(master, pitch_angle=15)# Pitch up 15 degrees

# Wait for a few seconds
time.sleep(2)

dmc.set_drone_pitch(master, pitch_rate=-0.3) # Pitch down continuously at 0.3 rad/s
    
time.sleep(3)
dmc.set_drone_pitch(master, pitch_rate=0) # Wait and then stop pitch motion