import socket
import sounddevice as sd
import numpy as np

# config
SERVER_IP = '169.231.9.9'  
SERVER_PORT = 42069 
BUFFER_SIZE = 1024  #packet size

FS = 16000  # Sampling rate
CHANNELS = 1  # Mono
DURATION = 0.1  # Duration of each audio packet capture

def record_and_send_audio():
    """Records audio in chunks and sends it to the server in real time."""
    print(f"Connecting to server at {SERVER_IP}:{SERVER_PORT}...")
    
    # connect to server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Connected to server.")
        
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
        finally:
            print("Closing connection.")
            client_socket.close()

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
                
                # Play the audio data
                sd.play(audio_data, samplerate=FS)
                sd.wait()  # Wait until playback is complete
                
                # Remove the played portion from the buffer
                audio_buffer = audio_buffer[FS * 2 * CHANNELS:]
    
    except Exception as e:
        print(f"Error during audio playback: {e}")

if __name__ == "__main__":
    try:
        record_and_send_audio()
    except Exception as e:
        print(f"Failed to start: {e}")
