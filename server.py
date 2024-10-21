import socket
import wave
import pyaudio
import os
from TTS.api import TTS
from TTS.utils.realtime import RealtimeTTS
import time
import numpy as np
import torch
import struct
from gpt4all import GPT4All
import whisper
from langchain.chains.conversation.memory import ConversationSummaryMemory
from RealtimeTTS import CoquiEngine, TextToAudioStream

#TODO
# Very important: 
# We're moving over to Ministral 3b or 8b. Check for quantized models, or load using fp16 or q8 quantization.
# We can use 8b if we have quantized versions of the models
# https://huggingface.co/mistralai/Ministral-8B-Instruct-2410

# Add memory for TTS
# Change LLM loading to transformer based
# Load models with fp16 or q8 quantization(LLM is already quantized, no need to do that one)
# YOLOv10 is fine and already uses minimal resources
# WIP: Use xtts_v2 streaming and stream the audio packets back as they're being generated
# which will hopefully decrease latency

# Configure the server
HOST = '0.0.0.0' 
PORT = 8080     

# Audio settings
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16 #jiayou!!!!!
CHANNELS = 1
RATE = 16000 

SILENCE_THRESHOLD = 500  # Change this value as we see fit
SILENCE_TIMEOUT = 1

device = "cuda" if torch.cuda.is_available() else "cpu"

transcription_model = whisper.load_model("base").to("cuda")

# Initialize TTS 
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to(device)

# Initialize streaming TTS
engine = CoquiEngine()
stream = TextToAudioStream(engine)

def load_templates(filename):
    templates = {}
    with open(filename, 'r') as file:
        current_key = None
        current_value = []
        for line in file:
            line = line.strip()
            if line.startswith('#'):
                if current_key:
                    templates[current_key] = ' '.join(current_value).replace('\\n', '\n')
                current_key = line[1:].strip()
                current_value = []
            else:
                current_value.append(line)
        if current_key:
            templates[current_key] = ' '.join(current_value).replace('\\n', '\n')
    return templates

# Set system templates
templates = load_templates('Chat_Bot_Templates.txt')
system_template = templates.get('default_paimon')
model = GPT4All(model_name='mistral-7b-openorca.Q5_K_M.gguf', allow_download=False,device="cuda", model_path= 'models/')# Can set the allow_download to false if you want to run it locally
prompt_template = 'Traveler: {0}\nPaimon: '


def generate_response(user_input):
    with model.chat_session(system_template, prompt_template):
        # Can force lower max token count(i.e. 100) so that responses are shorter and are faster to generate
        # Change temperature(temp) for more creative responses. Top_k, top_p, min_p, and repeat_penalty are all hyperparameters
        # Read documentation for further reference. 
        # https://docs.gpt4all.io/gpt4all_python/ref.html#gpt4all.gpt4all.GPT4All.generate
        response = model.generate(user_input, max_tokens=450, temp=1.1, top_k = 80, top_p = 0.85, min_p = 0.045, repeat_penalty = 2.1, n_batch=16)
        response_automated = f"{response}"
        return response_automated

def generate_and_stream_tts(client_socket, text):
    tts_text = generate_response(text)
    print(f"Generating TTS for: {tts_text}")
    
    stream.feed(tts_text)

    def audio_stream_generator():
        while True:
            audio_chunk = stream.get_next_chunk()  # find documentation to get next chunk. might be cooked
            if audio_chunk is None:
                break   
            yield audio_chunk

    # Stream and send the generated audio data chunk by chunk
    for audio_chunk in audio_stream_generator():
        # Send audio chunk
        client_socket.sendall(audio_chunk)
    
    print("Finished streaming audio.")
    
# Function to generate TTS and send back audio
def generate_and_send_tts(client_socket, text):
    tts_text = generate_response(text)
    print(f"Generating TTS for: {tts_text}")
    
    # Generate TTS audio 
    # Here when we generate TTS audio we can 
    wav_bytes = tts.tts_to_file(text=tts_text, speaker_wav=["emily1.wav", "IMG_1306.wav", "IMG_1307.wav", "IMG_1308.wav","IMG_1309.wav","IMG_1310.wav","IMG_1313.wav","IMG_1314.wav","IMG_1315.wav"], language="en", file_path="balsshd.wav")
    
    # Open the wav file and send it back
    wf = wave.open('response.wav', 'rb')
    
    while True:
        data = wf.readframes(CHUNK_SIZE)
        if not data:
            break
        client_socket.sendall(data)
    
    wf.close()
    os.remove('response.wav')  # Clean up

def calculate_rms(data):
    # Unpack the byte data into an array of integers
    int_data = np.frombuffer(data, dtype=np.int16)
    
    # Calculate RMS
    rms = np.sqrt(np.mean(int_data ** 2))
    return rms

def handle_client(client_socket):
    print("Client connected")

    # File to store data
    wf = wave.open('received_audio.wav', 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.get_sample_size(FORMAT))
    wf.setframerate(RATE)

    silence_duration = 0
    start_time = time.time()

    try:
        while True:
            data = client_socket.recv(CHUNK_SIZE)
            
            if data:
                # Check audio levels
                rms = calculate_rms(data)
                print(f"Current RMS Level: {rms}")

                # Reset silence timer as long as data is being received
                silence_duration = 0
                start_time = time.time()
                wf.writeframes(data)

                # Check if RMS level is above the silence threshold
                if rms < SILENCE_THRESHOLD:
                    print("Silence detected, processing audio...")
                    wf.close()
                    
                    # Process the audio file
                    AUDIO_FILE = "received_audio.wav"
                    transcribed_text = transcription_model.transcribe(AUDIO_FILE)

                    # Send back a TTS response
                    generate_and_send_tts(client_socket, transcribed_text)

                    # Reopen the file for further incoming audio data
                    wf = wave.open('received_audio.wav', 'wb')
                    wf.setnchannels(CHANNELS)
                    wf.setsampwidth(pyaudio.get_sample_size(FORMAT))
                    wf.setframerate(RATE)

                    # Reset silence timer and start listening for new speech
                    silence_duration = 0
                    start_time = time.time()

            else:
                # No data received
                silence_duration = time.time() - start_time

                # Check if silence has been long enough to consider end of speech
                if silence_duration >= SILENCE_TIMEOUT:
                    print("Silence detected, processing audio...")
                    wf.close()
                    
                    # Process the audio file
                    AUDIO_FILE = "received_audio.wav"
                    transcribed_text = transcription_model.transcribe(AUDIO_FILE)

                    # Send back a TTS response
                    generate_and_send_tts(client_socket, transcribed_text)

                    # Reopen the file for further incoming audio data
                    wf = wave.open('received_audio.wav', 'wb')
                    wf.setnchannels(CHANNELS)
                    wf.setsampwidth(pyaudio.get_sample_size(FORMAT))
                    wf.setframerate(RATE)

                    # Reset silence timer and start listening for new speech
                    silence_duration = 0
                    start_time = time.time()

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
