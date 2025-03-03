import socket
import wave
import pyaudio
import os
from TTS.api import TTS
#from TTS.utils.realtime import RealtimeTTS
import time
import numpy as np
import torch
import struct
from gpt4all import GPT4All
import whisper
from langchain.chains.conversation.memory import ConversationSummaryMemory
from RealtimeTTS import CoquiEngine, TextToAudioStream
HOST = '0.0.0.0'  
PORT = 42069   

# Audio settings
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16 #jiayou!!!!!
CHANNELS = 1
RATE = 16000 

SILENCE_THRESHOLD = 15  # Change this value as we see fit
SILENCE_TIMEOUT = 1
device = "cuda" if torch.cuda.is_available() else "cpu"

transcription_model = whisper.load_model("base").to("cuda")
def handle_client(client_socket):
    print("Client connected")

    # File to store data
    wf = wave.open('received_audio_test.wav', 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    print("wf file set")
    received_data = b''

    silence_duration = 0
    start_time = time.time()
    time.sleep(2)

    try:
        while True:
            #header = client_socket.recv(8).decode().strip()
            header = False
            data = client_socket.recv(CHUNK_SIZE)
            received_data += data
            #print(data)
            #if header == "CHECK":
            if header:
                # Im too tired to get this shit to work
                x, y = data.split()
                #check_follow_true(client_socket, float(x), float(y))
            else:
                if data:
                    # Check audio levels
                    if len(data) == 0:
                        print("Invalid data in int_data. Skipping RMS calculation.")
                        rms = 500  # or handle accordingly
                    else:
                        #rms = calculate_rms(data)
                        #print(f"Current RMS Level: {rms}")
                        print("Balls")
                        silence_duration = 0

                    # Reset silence timer as long as data is being received
                    #silence_duration = 0
                    #start_time = time.time()
                    if len(received_data) >= 20000:
                        wf.writeframes(received_data)
                        received_data = b''

                    # Check if RMS level is above the silence threshold
                    #if rms < SILENCE_THRESHOLD:
                    if time.time() - start_time >= 7:
                        print("Silence detected, processing audio...")
                        wf.close()
                        
                        # Process the audio file
                        AUDIO_FILE = "received_audio_test.wav"
                        transcribed_text = transcription_model.transcribe(AUDIO_FILE)
                        print(f"Transcription: " + transcribed_text["text"])
                        if transcribed_text["text"] == "":
                            print("No text")
                        else:
                            # Send back a TTS response
                            #generate_and_send_tts(client_socket, transcribed_text)
                            print("LOL")

                        # Reopen the file for further incoming audio data
                        wf = wave.open('received_audio_test.wav', 'wb')
                        wf.setnchannels(CHANNELS)
                        wf.setsampwidth(pyaudio.get_sample_size(FORMAT))
                        wf.setframerate(RATE)

                        # Reset silence timer and start listening for new speech
                        silence_duration = 0
                        start_time = time.time()

                else:
                    # No data received
                    print("No data received.")
                    silence_duration = time.time() - start_time

                    # Check if silence has been long enough to consider end of speech
                    if silence_duration >= SILENCE_TIMEOUT:
                        print("Silence detected, processing audio...")
                        wf.close()
                        
                        # Process the audio file
                        AUDIO_FILE = "received_audio_test.wav"
                        transcribed_text = transcription_model.transcribe(AUDIO_FILE)
                        print(f"Transcription: " + transcribed_text["text"])

                        # Send back a TTS response
                        #generate_and_send_tts(client_socket, transcribed_text)

                        # Reopen the file for further incoming audio data
                        wf = wave.open('received_audio_test.wav', 'wb')
                        wf.setnchannels(CHANNELS)
                        wf.setsampwidth(pyaudio.get_sample_size(FORMAT))
                        wf.setframerate(RATE)

                        # Reset silence timer and start listening for new speech
                        silence_duration = 0
                        start_time = time.time()

    except KeyboardInterrupt:
        print("\nServer is shutting down.")

    finally:
        wf.close()
        client_socket.close()
        print("Client disconnected")

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"Server listening on {HOST}:{PORT}")

        while True:
            client_socket, addr = server_socket.accept()
            print(f"Connection from {addr}")
            handle_client(client_socket)

if __name__ == '__main__':
    start_server()
