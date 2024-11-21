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
                sd.wait()  # Wait until the audio is fully recorded
                
                # Convert audio data to bytes
                audio_bytes = audio_chunk.tobytes()
                
                # Send audio data in chunks of BUFFER_SIZE
                for i in range(0, len(audio_bytes), BUFFER_SIZE):
                    packet = audio_bytes[i:i + BUFFER_SIZE]
                    client_socket.sendall(packet)
        
        except KeyboardInterrupt:
            print("\nStreaming stopped.")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            print("Closing connection.")
            client_socket.close()

if __name__ == "__main__":
    try:
        record_and_send_audio()
    except Exception as e:
        print(f"Failed to start: {e}")
