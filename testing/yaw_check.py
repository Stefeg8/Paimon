from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:127.0.0.1:14550") #should be port 14550 but check 
master.wait_heartbeat()

