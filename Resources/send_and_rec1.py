import socket
import wave
import os

# Configuration
SERVER_IP = '169.231.9.9'  # Replace with the remote PC's IP address
SERVER_PORT = 42069        # Port to connect to

# File to send
AUDIO_FILE = 'hutao_sample_bot2.wav'   # Replace with your .wav file path

def send_audio(file_path, server_ip, server_port):
    # Create a socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            print("Connecting to the server...")
            client_socket.connect((server_ip, server_port))
            print("Connected to the server.")

            # Send file size
            file_size = os.path.getsize(file_path)
            client_socket.sendall(f"{file_size}".encode('utf-8'))
            client_socket.recv(1024)  # Wait for acknowledgment

            # Send audio file
            with open(file_path, 'rb') as audio_file:
                print("Sending file...")
                while chunk := audio_file.read(1024):
                    client_socket.sendall(chunk)
                print("File sent successfully.")
        except Exception as e:
            print(f"Error: {e}")

# Run the sender
send_audio(AUDIO_FILE, SERVER_IP, SERVER_PORT)
