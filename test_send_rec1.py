import socket

# Configuration
HOST = '0.0.0.0'           # Listen on all interfaces
PORT = 42069             # Port to listen on
OUTPUT_FILE = 'received_audio_test.wav'


def receive_audio(output_file, host, port):
    # Create a socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((host, port))
        server_socket.listen(1)
        print(f"Listening for connections on {host}:{port}...")

        conn, addr = server_socket.accept()
        with conn:
            print(f"Connection established with {addr}")

            # Receive file size
            file_size = int(conn.recv(1024).decode('utf-8'))
            conn.sendall(b'ACK')  # Acknowledge file size

            # Receive audio file
            with open(output_file, 'wb') as audio_file:
                print("Receiving file...")
                bytes_received = 0
                while bytes_received < file_size:
                    chunk = conn.recv(1024)
                    if not chunk:
                        break
                    audio_file.write(chunk)
                    bytes_received += len(chunk)
                print(f"File received and saved as {output_file}.")

# Run the receiver
receive_audio(OUTPUT_FILE, HOST, PORT)
