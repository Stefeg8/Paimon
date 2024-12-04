import socket
import sounddevice as sd
import numpy as np
import cv2
import threading
from ultralytics import YOLO
import queue
import serial
import time

print("Available devices:")
print(sd.query_devices())

device_id = "what the fuck is my speaker's id"

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
DURATION = 0.1  # Duration of each audio packet capture

model = YOLO("yolov10n.pt") 
class_names_list = None
with open('coco.names', 'r') as f:  
    class_names_list = [line.strip() for line in f.readlines()]

#arduino = serial.Serial('/dev/ttyUSB0', 9600)  # erm wtf
time.sleep(2) 

def dir_movement(x, y):
    """Handles actions when follow mode is active."""
    print(f"Performing follow action with coordinates: x={x}, y={y}")
    
    # Decide movement based on x and y
    if x >= 290 and x <= 350:
        command = 'F'  # Forward
    elif x<290:
        command = 'L'  # Left
    elif x > 350:
        command = 'R'  # Right
    else:
        command = 'S'  # Stop
    
    # Send command to Arduino
    arduino.write(command.encode())
    print(f"Sent command to Arduino: {command}")

def directions(x, y, client_socket):
    """Processes the detected coordinates and communicates with the server."""
    try:
        # Send the CHECK_FOLLOW message to the server
        message = f"CHECK_FOLLOW {x} {y}"
        client_socket.sendall(message.encode())
        print(f"Sent directions to server: {message}")
        
        # Wait for and process the follow status from the server
        data = client_socket.recv(1024).decode()
        if "follow:true" in data:
            print("Server confirmed: Follow mode active.")
            dir_movement(x, y)  # Call a function if follow is active
        elif "follow:false" in data:
            print("Server confirmed: Follow mode inactive.")
    except Exception as e:
        print(f"Error in directions function: {e}")

def record_and_send_audio(client_socket):
    """Records audio in chunks and sends it to the server in real time."""
    try:
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
            sd.wait()  # W]ait until the audio is fully recorded
            
            # Convert audio data to bytes
            audio_bytes = audio_chunk.tobytes()
            
            # Send audio data in chunks of BUFFER_SIZE
            for i in range(0, len(audio_bytes), BUFFER_SIZE):
                packet = audio_bytes[i:i + BUFFER_SIZE]
                client_socket.sendall(packet)
            receive_and_play_audio(client_socket)
    
    except KeyboardInterrupt:
        print("\nStreaming stopped.")
    except Exception as e:
        print(f"An error occurred: {e}")
def receive_and_play_audio(client_socket):
    """Receives audio data from the server and plays it in real time."""
    audio_buffer = bytearray()  # Buffer to accumulate audio data
    
    try:
        # Continuously receive audio data from the server
        while True:
            data = client_socket.recv(BUFFER_SIZE)
            if not data:
                break  # If no data is received, exit the loop
            
            # Append the received data to the buffer
            audio_buffer.extend(data)

            # If the buffer has enough data for a playback chunk
            if len(audio_buffer) >= FS * 2 * CHANNELS:
                # Convert the buffer into a NumPy array for playback
                audio_data = np.frombuffer(audio_buffer[:FS * 2 * CHANNELS], dtype=np.int16)
                
                # Play the audio data on the Bluetooth speaker
                sd.play(audio_data, samplerate=FS, device=device_id)  # Specify device
                sd.wait()  # Wait until playback is complete
                
                # Remove the played portion from the buffer
                audio_buffer = audio_buffer[FS * 2 * CHANNELS:]
    
    except Exception as e:
        print(f"Error during audio playback: {e}")
        
def capture_and_process_video():
    """Captures video frames, processes them, and sends them to the server."""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, VIDEO_FPS)

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            print("Cap reading")
            if not ret:
                print("broken")
                break
            
            # YOLO Object Detection
            results = model(frame)[0]
            cv2.imshow(frame)
            print(results)
            for result in results.boxes.data:
                x_min, y_min, x_max, y_max, confidence, class_id = result.tolist()
                if int(class_id) < len(class_names_list) and class_names_list[int(class_id)] == "person":
                    center_x = (x_min + x_max) / 2
                    center_y = (y_min + y_max) / 2
                    print(center_x,center_y)
                    directions(center_x, center_y)

    except Exception as e:
        print(f"Video streaming error: {e}")
    finally:
        cap.release()

def main():
    """Main function to start threads for audio and video streaming."""
    print(f"Connecting to server at {SERVER_IP}:{SERVER_PORT}...")
    while True:
        capture_and_process_video()
    # Connect to server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Connected to server.")
        while True:
            capture_and_process_video()
            # message = f"CHECK_FOLLOW"
            # print(message)
            # client_socket.sendall(message.encode())


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Failed to start: {e}")
