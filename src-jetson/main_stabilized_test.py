'''
Usage: Load onto Jetson or Raspberry Pi.
Functions: Contains audio gathering and sending functions as well as 
movement commands and YOLO inference. 
This version has thread safety, stabilized flight, auto takeoff, and auto landing on Ctrl+C.
'''

from inc import drone_move_cmds as dmc
from inc import pitch_calculation as pcalc
from inc import fallback as fb  # fallbacks, contain autolevel and autoland
from lidar import tfmini as ld
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R
import time
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
from scipy.io.wavfile import write
import os
import subprocess

resource_lock = threading.Lock()

# MAVLink connection
print("Initializing connection")
master = mavutil.mavlink_connection("COM4", baud=57600)
print("working...")
master.wait_heartbeat()
print(f"connection established with system {master.target_system}")
start_time = time.time()

# Video config
WIDTH = 1280
HEIGHT = 720
cmd = [
    'libcamera-vid', '--width', str(WIDTH), '--height', str(HEIGHT),
    '--codec', 'yuv420', '--nopreview', '-t', '0', '-o', '-'
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

# Global attitude variables
last_quat = None
last_thrust = None
send_setpoint = True

# Stabilization control

def stabilize_attitude():
    global last_quat, last_thrust
    MAX_PITCH = 20

    while True:
        with resource_lock:
            curr_roll, curr_pitch, curr_yaw = dmc.get_current_attitude(master)
            q_current = dmc.get_current_attitude_quaternion(master)
            distance, _ = ld.getTFminiData()

            pitch_increment = dmc.get_pitch_increment(distance)
            if distance < 50:
                q_hover = R.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()
                last_quat = q_hover
                last_thrust = 0.54
            elif pitch_increment != 0 and curr_pitch + pitch_increment <= MAX_PITCH:
                q_pitch_up = dmc.pitch_up_calc(q_current, pitch_increment)
                last_quat = q_pitch_up
                last_thrust = 0.56
            else:
                last_quat = q_current
                last_thrust = 0.54
        time.sleep(0.1)

def maintain_offboard_mode(master):
    while True:
        if send_setpoint and last_quat is not None:
            with resource_lock:
                dmc.set_attitude(master, last_quat, last_thrust)
        time.sleep(0.05)

# Main detection and tracking logic

def detect_and_follow():
    global last_quat, last_thrust
    target_bbox_height = 300  # Ideal height of person bbox in pixels
    gain = 0.05  # Gain factor to scale pitch adjustment

    while True:
        raw_frame = process.stdout.read(WIDTH * HEIGHT * 3 // 2)
        if not raw_frame:
            continue

        yuv = np.frombuffer(raw_frame, dtype=np.uint8).reshape((HEIGHT * 3 // 2, WIDTH))
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

        results = model.predict(bgr, verbose=False)
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()
            for box, cls in zip(boxes, classes):
                class_id = int(cls)
                label = class_names_list[class_id]
                if label.lower() == "person":
                    x1, y1, x2, y2 = box
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    height = int(y2 - y1)

                    with resource_lock:
                        curr_q = dmc.get_current_attitude_quaternion(master)

                        yaw_q = dmc.yaw_adjust_from_center(cx, WIDTH)
                        pitch_error = target_bbox_height - height
                        pitch_deg = gain * pitch_error
                        pitch_deg = np.clip(pitch_deg, -20, 20)

                        new_q = dmc.pitch_up_calc(curr_q, pitch_deg) if abs(pitch_deg) > 1 else curr_q

                        if yaw_q is not None:
                            yaw_euler = R.from_quat(yaw_q).as_euler('xyz', degrees=True)
                            new_q = R.from_euler('xyz', [0, pitch_deg, yaw_euler[2]], degrees=True).as_quat()

                        last_quat = new_q
                        last_thrust = 0.55

        time.sleep(0.1)

# Signal handler for graceful shutdown

def signal_handler(sig, frame):
    print("\nCtrl+C received. Initiating auto-landing...")
    fb.auto_land(master)
    print("Landed. Shutting down.")
    fb.system_shutdown(master)
    sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# Start threads
stabilization_thread = threading.Thread(target=stabilize_attitude, daemon=True)
stabilization_thread.start()

setpoint_thread = threading.Thread(target=maintain_offboard_mode, args=(master,), daemon=True)
setpoint_thread.start()

detection_thread = threading.Thread(target=detect_and_follow, daemon=True)
detection_thread.start()

# Perform auto takeoff
print("Performing auto takeoff...")
fb.auto_takeoff(master,start_time)
print("Takeoff complete. Entering control loop.")

# Keep program alive
while True:
    time.sleep(1)
