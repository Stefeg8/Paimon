import subprocess
import cv2
import numpy as np
from ultralytics import YOLO
from inc import drone_move_cmds as dmc
from inc import pitch_calculation as pcalc
from lidar import tfmini as ld
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R
import socket
import sounddevice as sd
import threading
import queue
import wave
import os
import time

# Set resolution
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
DURATION = 5  # Duration of each audio packet capture
FILENAME = "output_files/recorded_audio.wav"
model = YOLO("src-jetson/inc/yolov10n.pt")
class_names_list = None
with open('src-jetson/inc/coco.names', 'r') as f:
    class_names_list = [line.strip() for line in f.readlines()]
output_file = "tts_output.wav"

master = mavutil.mavlink_connection("udp:127.0.0.1:14550")  # Adjust port as needed
master.wait_heartbeat()

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

def capture_and_send_video_lib(client_socket):
    while True:
        frame = read_frame()
        if frame is None:
            break

        # Process frame with YOLO
        pipeline(frame)

    cv2.destroyAllWindows()
    process.terminate()

def record_and_send_audio1(client_socket):
    while True:
        try:
            audio_chunk = sd.rec(int(DURATION * FS), samplerate=FS, channels=CHANNELS, dtype='int16')
            sd.wait()
            write(FILENAME, FS, audio_chunk)

            file_size = os.path.getsize(FILENAME)
            client_socket.sendall(f"{file_size}".encode('utf-8'))
            client_socket.recv(1024)  # ACK

            with open(FILENAME, 'rb') as audio_file:
                while chunk := audio_file.read(1024):
                    client_socket.sendall(chunk)

            receive_audio(client_socket)  # Ensure this function doesn't block indefinitely

        except Exception as e:
            print(f"Error during recording or sending: {e}")
            client_socket.close()
            break

def main():
    print(f"Connecting to server at {SERVER_IP}:{SERVER_PORT}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Connected to server.")

        dmc.initialize_offboard(master)

        audio_thread = threading.Thread(target=record_and_send_audio1, args=(client_socket,))
        video_thread = threading.Thread(target=capture_and_send_video_lib, args=(client_socket,))
        audio_thread.start()
        video_thread.start()

        audio_thread.join()
        video_thread.join()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Failed to start: {e}")
