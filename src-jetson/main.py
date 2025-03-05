from inc import drone_move_cmds as dmc
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
FILENAME = "recorded_audio.wav" 
model = YOLO("yolov10n.pt") 
class_names_list = None
with open('coco.names', 'r') as f:  
    class_names_list = [line.strip() for line in f.readlines()]
output_file = "tts_output.wav"
arduino = serial.Serial('/dev/ttyACM0', 9600)  # erm wtf
time.sleep(2) 

def dir_movement(x, y):
    """Handles actions when follow mode is active."""
    print(f"Performing follow action with coordinates: x={x}, y={y}")
    
    # Decide movement based on x and y
    if x >=1500 and x <= 3000:
        command = 'F'  # Forward
    elif x<1500:
        command = 'L'  # Left
    elif x > 3000:
        command = 'R'  # Right
    else:
        command = 'S'  # Stop
    print(command)
    # Send command to Arduino
    arduino.write(command.encode())
    print(f"Sent command to Arduino: {command}")
    time.sleep(1)
    if arduino.in_waiting >0:
        ack = arduino.read(arduino.in_waiting).decode('utf-8').strip()
        #print(f"received ack {ack}")
    else:
        print("No ack")

def directions(x, y, client_socket):
    """Processes the detected coordinates and communicates with the server."""
    # deprecated
    try:
        # Send the CHECK_FOLLOW message to the server
        header = "CHECK".ljust(8)  # Pad to 8 bytes
        message = f"{header}{x} {y}"
        client_socket.sendall(message.encode())
        print(f"Sent directions to server: {message}")
        
        # Wait for and process the follow status from the server
        data = client_socket.recv(1024).decode()
        if "follow:true" in data:
            print("Server confirmed: Follow mode active.")
            #dir_movement(x, y)  # Call a function if follow is active
        elif "follow:false" in data:
            print("Server confirmed: Follow mode inactive.")
    except Exception as e:
        print(f"Error in directions function: {e}")

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

            # Convert audio data to bytes
            # audio_bytes = audio_chunk.tobytes()
            
            # # Send audio data in chunks of BUFFER_SIZE
            # #header = "AUDIO".ljust(8)  # Pad to 8 bytes
            # for i in range(0, len(audio_bytes), BUFFER_SIZE):
            #     #packet = header.encode() + audio_bytes[i:i + BUFFER_SIZE]
            #     packet = audio_bytes[i:i + BUFFER_SIZE]
            #     client_socket.sendall(packet)
            #     print("Packets sent")
            # #receive_and_play_audio(client_socket)
    
    #except KeyboardInterrupt:
        #print("\nStreaming stopped.")
    #except Exception as e:
        #print(f"An error occurred: {e}")

def play_audio(client_socket):
    wf = wave.open("tts_output.wav", 'rb')

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
        with open("tts_output.wav", 'wb') as audio_file:
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
def receive_and_play_audio(client_socket):
    """Receives audio data from the server and plays it in real time."""
    audio_buffer = bytearray()  # Buffer to accumulate audio data
    time1 = time.time()
    trytoreceive=True
    print("Receive and play being called")
    try:
        # Continuously receive audio data from the server
        while trytoreceive:
            data = client_socket.recv(BUFFER_SIZE)
            print("data being received")
            if not data:
                print("No data")
                break  # If no data is received, exit the loop
            
            # Append the received data to the buffer
            audio_buffer.extend(data)

            # If the buffer has enough data for a playback chunk
            if len(audio_buffer) >= FS * 2 * CHANNELS:
                # Convert the buffer into a NumPy array for playback
                audio_data = np.frombuffer(audio_buffer[:FS * 2 * CHANNELS], dtype=np.int16)
                
                # Play the audio data on the Bluetooth speaker
                sd.play(audio_data, samplerate=FS)  # Specify device
                sd.wait()  # Wait until playback is complete
                
                # Remove the played portion from the buffer
                audio_buffer = audio_buffer[FS * 2 * CHANNELS:]
    
    except Exception as e:
        print(f"Error during audio playback: {e}")
        
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
                x_deg_calc,y_deg_calc = dmc.pixels_to_degrees((x_min+x_max)/2, (y_min+y_max)/2)
                quaternion = dmc.euler_to_quaternion_quaternion(0,y_deg_calc, x_deg_calc)
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

    print("Image capture complete.")

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
