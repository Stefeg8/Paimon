'''
Usage: Load onto Jetson or Raspberry Pi.
Functions: Contains audio gathering and sending functions as well as 
movement commands and YOLO inference. 
This version has thread safety. Otherwise, use main2
'''

from inc import drone_move_cmds as dmc
from inc import pitch_calculation as pcalc
from inc import fallback as fb # fallbacks, contain autolevel and autoland. if need to autoland, then autolevel->autoland
from lidar import tfmini as ld
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R
import time

print("Initializing connection")
master = mavutil.mavlink_connection("COM4", baud=57600)
print("working...")
master.wait_heartbeat()
print(f"connection established with system {master.target_system}")
start_time = time.time()

import signal
import sys
import socket
import sounddevice as sd
import numpy as np
import cv2
import threading
from ultralytics import YOLO
import queue
import wave
import serial
import time
from scipy.io.wavfile import write
import os
import subprocess

resource_lock = threading.Lock()

print("Available devices:")
print(sd.query_devices())


# config
SERVER_IP = '169.231.9.9'  
SERVER_PORT = 42069 
BUFFER_SIZE = 1024  #packet size

# Video config
WIDTH = 1280
HEIGHT = 720

cmd = [
    'libcamera-vid',
    '--width', str(WIDTH),
    '--height', str(HEIGHT),
    '--codec', 'yuv420',
    '--nopreview',
    '-t', '0',
    '-o', '-'
]
process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=WIDTH * HEIGHT * 3 // 2)


FS = 16000  # Sampling rate
CHANNELS = 1  # Mono
DURATION = 7  # Duration of each audio packet capture
FILENAME = "recorded_audio.wav" 
model = YOLO("src-jetson/inc/yolov10n.pt") 
class_names_list = None
with open('src-jetson/inc/coco.names', 'r') as f:  
    class_names_list = [line.strip() for line in f.readlines()]
output_file = "tts_output.wav"
time.sleep(2) 

# Global variables for last known attitude and thrust
last_quat = None
last_thrust = None
send_setpoint = False  

# Function to continuously maintain OFFBOARD mode
def maintain_offboard_mode(master):
    while True:
        if send_setpoint and last_quat is not None:
            # Use the lock to ensure thread-safe access to shared resources
            with resource_lock:
                dmc.set_attitude(master, last_quat, last_thrust)
        time.sleep(0.05)  # Send setpoint every 50ms (20 Hz)

def read_frame():
    yuv_size = WIDTH * HEIGHT * 3 // 2
    yuv_frame = process.stdout.read(yuv_size)
    if len(yuv_frame) != yuv_size:
        return None
    yuv_np = np.frombuffer(yuv_frame, dtype=np.uint8).reshape((HEIGHT * 3 // 2, WIDTH))
    bgr = cv2.cvtColor(yuv_np, cv2.COLOR_YUV2BGR_I420)
    return bgr

def pipeline(frame):
    results = model(frame)[0]
    for result in results.boxes.data:
        x_min, y_min, x_max, y_max, confidence, class_id = result.tolist()
        if int(class_id) < len(class_names_list) and class_names_list[int(class_id)] == "person":
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
            directions(center_x, center_y)


def directions(x_deg_calc, y_deg_calc):
    with resource_lock:
        curr_roll, curr_pitch, curr_yaw = dmc.get_current_attitude(master)
        q_current = dmc.get_current_attitude_quaternion(master) 
        dir_cam = dmc.angles_to_direction_vector(x_deg_calc, y_deg_calc)
        dir_world = dmc.camera_to_world_vector(dir_cam, q_current)
        q_target = dmc.look_rotation(dir_world)
        q_next = dmc.slerp_rotation(q_current, q_target, t=0.1) 
        
        # lidar dist
        distance, strength = ld.getTFminiData()

        # Distance thresholds for reference(in cm)
        SAFE_DISTANCE = 270
        CRITICAL_DISTANCE = 240
        MAX_PITCH = 20

        pitch_increment = dmc.get_pitch_increment(distance)
        hover_quat = R.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()
        
        if distance <= CRITICAL_DISTANCE:
            dmc.set_attitude(master, hover_quat, thrust=0.54)
            with resource_lock:
                dmc.set_attitude(master, hover_quat, thrust=0.54)
                global last_quat, last_thrust
                last_quat = hover_quat
                last_thrust = 0.54
            return

        if pitch_increment == 0:
            if distance > SAFE_DISTANCE:
                dmc.set_attitude(master, q_next, thrust=0.56)
                with resource_lock:
                    global last_quat, last_thrust
                    last_quat = q_next
                    last_thrust = 0.56
                return
            else:
                dmc.set_attitude(master, hover_quat, thrust=0.54)
                with resource_lock:
                    global last_quat, last_thrust
                    last_quat = hover_quat
                    last_thrust = 0.54
                return
        else:
            if curr_pitch + pitch_increment <= MAX_PITCH:
                q_pitch_up = dmc.pitch_up_calc(q_current, pitch_increment)
                dmc.set_attitude(master, q_pitch_up, thrust=0.56)
                with resource_lock:
                    global last_quat, last_thrust
                    last_quat = q_pitch_up
                    last_thrust = 0.56
                return
            else:
                dmc.set_attitude(master, q_current, thrust=0.54)
                with resource_lock:
                    global last_quat, last_thrust
                    last_quat = q_current
                    last_thrust = 0.54
                return
def capture_and_send_video_lib(client_socket):
    while True:
        frame = read_frame()
        if frame is None:
            break

        # Process frame with YOLO
        pipeline(frame)

    cv2.destroyAllWindows()
    process.terminate()

def start_offboard_thread(master):
    thread = threading.Thread(target=maintain_offboard_mode, args=(master,))
    thread.daemon = True
    thread.start()

def send_heartbeat(master):
    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(1)

def ramp_up_drone(master):
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
    print("sending pre-offboard attitude targets")
    for _ in range(20):
        master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask = 0b00000111  # ignore body rates, use quaternion + thrust
    ,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=0.0
        )
        time.sleep(0.05)
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
    
    ramp_duration = 10
    takeoff_thrust = 0.54
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
            type_mask = 0b00000111  # ignore body rates, use quaternion + thrust
            ,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=thrust
        )
        time.sleep(0.05)
    
    print("hovering")
    
    hold_duration = 2
    hover_end = time.time() + hold_duration
    global last_quat, last_thrust
    last_quat = [1, 0, 0, 0]
    last_thrust = 0.54
    while time.time() < hover_end:
        master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask = 0b00000111  # ignore body rates, use quaternion + thrust
            ,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=0.54
        )
        time.sleep(0.05)
    start_offboard_thread(master)
    

def shutdown_procedure():
    global process, master

    print("\n[!] Shutdown initiated. Cleaning up...")
        # Ramp down
    print("ramping down thrust")
    time2 = time.time()
    duration = 10  # seconds

    while time.time() - time2 < duration:
        master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),  # time_boot_ms
            master.target_system,
            master.target_component,
            type_mask=0b00000111,  # ignore body rates, use quaternion + thrust
            q=[1, 0, 0, 0],  # no rotation
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=0.51
        )
        time.sleep(0.05)
    # Disarm the drone safely
    try:
        print("[>] Disarming drone...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # disarm
            0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        print("[✓] Drone disarmed.")
    except Exception as e:
        print(f"[X] Error disarming drone: {e}")

    # Terminate camera subprocess
    if process:
        print("[>] Terminating camera process...")
        process.terminate()
        process.wait()
        print("[✓] Camera process terminated.")

    # Optional: Clean up any GPIO, files, etc.
    print("[✓] All systems safely shut down.")
    sys.exit(0)

# Attach signal handler
def signal_handler(sig, frame):
    shutdown_procedure()

signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
signal.signal(signal.SIGTERM, signal_handler)  # kill PID

def main():
    """Main function to start threads for audio and video streaming."""
    print(f"Connecting to server at {SERVER_IP}:{SERVER_PORT}...")
    
    # Connect to server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Connected to server.")

        #initialize offboard mode
        #dmc.initialize_offboard(master)
        ramp_up_drone(master)
        
        #receive_thread = threading.Thread(target=receive_audio, args=(client_socket,))
        video_thread = threading.Thread(target=capture_and_send_video_lib, args=(client_socket,))
        #receive_thread.start()
        video_thread.start()
        #receive_thread.join()
        video_thread.join()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        shutdown_procedure()
    except Exception as e:
        print(f"Failed to start: {e}")
        shutdown_procedure()
