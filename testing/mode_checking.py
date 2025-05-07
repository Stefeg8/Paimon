from pymavlink import mavutil

def check_current_mode(master):
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if msg:
        print(f"Current mode: {msg.custom_mode} (should match PX4's packed mode)")
    else:
        print("No heartbeat received")

if __name__ == "__main__":
    master = mavutil.mavlink_connection('COM4', baud=57600)
    check_current_mode(master)