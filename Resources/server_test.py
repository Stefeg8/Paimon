import socket
import wave
import pyaudio
import os
from TTS.api import TTS
from RealtimeTTS import CoquiEngine, TextToAudioStream
import torch
from gpt4all import GPT4All
import whisper
from langchain.chains.conversation.memory import ConversationSummaryMemory
import time
import numpy as np
import struct

# Server settings
HOST = '0.0.0.0'
PORT = 42069

# Audio settings
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

SILENCE_THRESHOLD = 500
SILENCE_TIMEOUT = 1

device = "cuda" if torch.cuda.is_available() else "cpu"

# Load models
transcription_model = whisper.load_model("base").to("cuda")
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to(device)

# Initialize streaming TTS engine
def init_tts():
    engine = CoquiEngine()
    stream = TextToAudioStream(engine)
    return stream

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

templates = load_templates('paimon_templates.txt')
system_template = templates.get('default_paimon')

model = GPT4All(model_name='mistral-7b-openorca.Q5_K_M.gguf', allow_download=False, device="cuda", model_path='models/')
prompt_template = 'Traveler: {0}\nPaimon: '

def generate_response(user_input):
    with model.chat_session(system_template, prompt_template):
        response = model.generate(user_input, max_tokens=450, temp=1.1, top_k=80, top_p=0.85, min_p=0.045, repeat_penalty=2.1, n_batch=16)
        response_automated = f"{response}"
        return response_automated

def generate_and_stream_tts(client_socket, text, stream):
    tts_text = generate_response(text)
    print(f"Generating TTS for: {tts_text}")
    
    stream.feed(tts_text)

    def audio_stream_generator():
        while True:
            audio_chunk = stream.get_next_chunk()
            if audio_chunk is None:
                break
            yield audio_chunk

    for audio_chunk in audio_stream_generator():
        client_socket.sendall(audio_chunk)
    
    print("Finished streaming audio.")

def generate_and_send_tts(client_socket, text):
    tts_text = generate_response(text)
    print(f"Generating TTS for: {tts_text}")
    
    tts.tts_to_file(text=tts_text, speaker_wav=["emily1.wav"], language="en", file_path="response.wav")
    
    wf = wave.open('response.wav', 'rb')
    
    while True:
        data = wf.readframes(CHUNK_SIZE)
        if not data:
            break
        client_socket.sendall(data)
    
    wf.close()
    os.remove('response.wav')

def calculate_rms(data):
    int_data = np.frombuffer(data, dtype=np.int16)
    rms = np.sqrt(np.mean(int_data ** 2))
    return rms

def handle_client(client_socket, stream):
    print("Client connected")

    try:
        while True:
            data = client_socket.recv(CHUNK_SIZE)
            print(data)
    finally:
        client_socket.close()
        print("Client disconnected")

def start_server():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((HOST, PORT))
            server_socket.listen(1)
            print(f"Server listening on {HOST}:{PORT}")

            while True:
                client_socket, addr = server_socket.accept()
                print(f"Connection from {addr}")
                handle_client(client_socket, stream)
    except KeyboardInterrupt:
        print("\nServer is shutting down.")

if __name__ == '__main__':
    # Initialize TTS engine and start server inside the main guard
    stream = init_tts()
    start_server()
