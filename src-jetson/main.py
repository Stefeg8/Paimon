'''
Usage: Load onto Jetson or Raspberry Pi.
Functions: Contains audio gathering and sending functions as well as 
movement commands and YOLO inference. 
'''

from inc import drone_move_cmds as dmc
from inc import pitch_calculation as pcalc
from lidar import tfmini as ld
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

# Example: Turn 30 degrees right
dmc.set_yaw_angle(master, yaw_angle=30)


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

print("Available devices:")
print(sd.query_devices())


# config
SERVER_IP = '169.231.9.9'  
SERVER_PORT = 42069 
BUFFER_SIZE = 1024  #packet size

# Video config
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
VIDEO_BUFFER_SIZE = 4096
VIDEO_FPS = 3

FS = 16000  # Sampling rate
CHANNELS = 1  # Mono
DURATION = 5  # Duration of each audio packet capture
FILENAME = "output_files/recorded_audio.wav" 
model = YOLO("inc/yolov10n.pt") 
class_names_list = None
with open('inc/coco.names', 'r') as f:  
    class_names_list = [line.strip() for line in f.readlines()]
output_file = "output_files/tts_output.wav"
time.sleep(2) 

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


def record_and_send_audio(client_socket):
    """Records audio in chunks and sends it to the server in real time."""
    #try:
        # Continuously record and send audio
    print("Starting audio streaming...")
    while True:
        # Record a small chunk of audio
        audio_chunk = sd.rec(
            int(DURATION * FS), 
            samplerate=FS, 
            channels=CHANNELS, 
            dtype='int16'
        )
        sd.wait()  # Wait until the audio is fully recorded
        write(FILENAME, FS, audio_chunk)
        try:
            # Send file size
            file_size = os.path.getsize(FILENAME)
            client_socket.sendall(f"{file_size}".encode('utf-8'))
            client_socket.recv(1024)  # Wait for acknowledgment

            # Send audio file
            with open(FILENAME, 'rb') as audio_file:
                print("Sending file...")
                while chunk := audio_file.read(1024):
                    client_socket.sendall(chunk)
                print("File sent successfully.")
                receive_audio(client_socket)
        except Exception as e:
            print(f"Error: {e}")

def play_audio(client_socket):
    wf = wave.open("output_files/tts_output.wav", 'rb')

    # Read audio data from file
    frames = wf.readframes(wf.getnframes())
    # Convert bytes to numpy array
    audio_data = np.frombuffer(frames, dtype=np.int16)

    # Play the audio
    sd.play(audio_data, wf.getframerate())

    # Wait until audio finishes playing
    sd.wait()
    #receive_audio(client_socket)
    record_and_send_audio(client_socket)

def receive_audio(client_socket):
    with client_socket:
        print("Connection established")
        # Receive file size
        file_size = int(client_socket.recv(1024).decode('utf-8'))
        client_socket.sendall(b'ACK')  # Acknowledge file size

        # Receive audio file
        with open("output_files/tts_output.wav", 'wb') as audio_file:
            print("Receiving file...")
            bytes_received = 0
            while bytes_received < file_size:
                chunk = client_socket.recv(1024)
                if not chunk:
                    break
                audio_file.write(chunk)
                bytes_received += len(chunk)
            print(f"File received and saved as {output_file}.")
        play_audio(client_socket)
        
def capture_and_send_video(client_socket):
    """Captures video frames, processes them, and sends them to the server."""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, VIDEO_FPS)

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # YOLO Object Detection
            results = model(frame)[0]
            for result in results.boxes.data:
                x_min, y_min, x_max, y_max, confidence, class_id = result.tolist()
                if int(class_id) < len(class_names_list) and class_names_list[int(class_id)] == "person":
                    center_x = (x_min + x_max) / 2
                    center_y = (y_min + y_max) / 2
                    directions(center_x, center_y)

    except Exception as e:
        print(f"Video streaming error: {e}")
    finally:
        cap.release()

def run_yolo_on_image(image_path):
    try:
        frame = cv2.imread(image_path)
        if frame is None:
            print(f"Failed to read image {image_path}")
            return

        # Resize frame if needed (optional, YOLO will auto-resize internally)
        # resized_frame = cv2.resize(frame, (640, 480))

        # YOLO inference
        print("Yolo inference rn")
        results = model(frame)[0]
        for result in results.boxes.data:
            x_min, y_min, x_max, y_max, confidence, class_id = result.tolist()
            print("Inferenced")
            if int(class_id) < len(class_names_list):
                label = class_names_list[int(class_id)]
                print(f"Detected {label} with confidence {confidence:.2f} at [{x_min}, {y_min}, {x_max}, {y_max}]")
                x_deg_calc,y_deg_calc = pcalc.pixels_to_degrees((x_min+x_max)/2, (y_min+y_max)/2)
                quaternion = dmc.euler_to_quaternion(0,y_deg_calc, x_deg_calc) 
                norm_quaternion = dmc.normalize_quaternion(dmc.euler_to_quaternion(0,y_deg_calc, x_deg_calc))
                curr_roll, curr_yaw, curr_pitch = dmc.get_current_attitude(master)
                q_current = dmc.get_current_attitude_quaternion(master) # calculating quaternion to pass to set_attitude 
                dir_cam = dmc.angles_to_direction_vector(x_deg_calc, y_deg_calc)
                dir_world = dmc.camera_to_world_vector(dir_cam, q_current)
                q_target = dmc.look_rotation(dir_world)
                q_next = dmc.slerp_rotation(q_current, q_target, t=0.1) 
                distance, strength = ld.getTFminiData()
                if(distance>160):
                    dmc.set_attitude(master, q_next)
                else:
                    q_pitch_up = dmc.pitch_up_calc(q_current,10)
                    dmc.set_attitude(master, q_pitch_up)
                # send movement command here
                # dir_movement((x_min+x_max)/2,100)  # deprecated

    except Exception as e:
        print(f"Error during YOLO inference: {e}")


def capture_and_send_video_lib(client_socket):
    duration=5
    fps=2
    output_dir="captured_images"
    os.makedirs(output_dir, exist_ok=True)  # Create the output directory if it doesn't exist
    interval = 1.0 / fps  # Time between captures in seconds
    start_time = time.time()

    while True:
        timestamp = int(time.time() * 1000)  # Unique timestamp for filename
        image_path = os.path.join(output_dir, f"frame_{timestamp}.jpg")
        
        # Capture the image using libcamera-still
        cmd = ["libcamera-still", "-n", "-o", image_path]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print(f"Captured image: {image_path}")
        run_yolo_on_image(image_path)  # Run YOLO inference on the captured image

        time.sleep(interval)  # Maintain consistent FPS

def main():
    """Main function to start threads for audio and video streaming."""
    print(f"Connecting to server at {SERVER_IP}:{SERVER_PORT}...")
    
    # Connect to server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Connected to server.")
        
        # Create and start threads
        audio_thread = threading.Thread(target=record_and_send_audio1, args=(client_socket,)) 
        #receive_thread = threading.Thread(target=receive_audio, args=(client_socket,))
        video_thread = threading.Thread(target=capture_and_send_video_lib, args=(client_socket,))
        audio_thread.start()
        #receive_thread.start()
        video_thread.start()

        # Wait for threads to complete
        audio_thread.join()
        #receive_thread.join()
        video_thread.join()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Failed to start: {e}")
