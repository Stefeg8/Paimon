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

# Initialize TTS 
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to(device)

follow = False

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
templates = load_templates('paimon_templates.txt')
system_template = templates.get('default_paimon')
model = GPT4All(model_name='mistral-7b-openorca.Q5_K_M.gguf', allow_download=False,device="cuda", model_path= 'models/')# Can set the allow_download to false if you want to run it locally
prompt_template = 'Traveler: {0}\nPaimon: '


def generate_response(user_input):
    with model.chat_session(system_template, prompt_template): # start this as the server boots up
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
    if "follow" in tts_text:
        follow = True
    print(f"Generating TTS for: {tts_text}")
    
    # Generate TTS audio 
    # Here when we generate TTS audio we can 
    tts.tts_to_file(text=tts_text, speaker_wav=["emily1.wav", "IMG_1306.wav", "IMG_1307.wav", "IMG_1308.wav","IMG_1309.wav","IMG_1310.wav","IMG_1313.wav","IMG_1314.wav","IMG_1315.wav"], language="en", file_path="response.wav")
    
    # Open the wav file and send it back
    wf = wave.open('response.wav', 'rb')
    
    while True:
        data = wf.readframes(CHUNK_SIZE)
        if not data:
            break
        client_socket.sendall(data)
    
    wf.close()
    os.remove('response.wav')  

def calculate_rms(data):
    if len(data) == 0:
        return 500
    
    # Unpack the byte data into an array of integers
    int_data = np.frombuffer(data, dtype=np.int16)
    
    if len(int_data) == 0:
        return 500
    
    # Check for non-finite values
    if not np.all(np.isfinite(int_data)):
        return 500
    
    # Calculate RMS
    rms = np.sqrt(np.mean(int_data ** 2))
    
    if np.isnan(rms):
        print("RMS calculation resulted in NaN.")
        return 500
    
    return rms

def check_follow_true(client_socket, x, y):
    """Handle the follow logic when the command is received."""
    global follow
    if follow:
        print(f"Following coordinates: x={x}, y={y}")
        response = "follow:true"  # Send 'true' if following
    else:
        print("Not following.")
        response = "follow:false"  # Send 'false' if not following

    # Send the response back to the Raspberry Pi
    try:
        client_socket.sendall(response.encode())
    except Exception as e:
        print(f"Error sending follow status: {e}")

def handle_client(client_socket):
    print("Client connected")

    # File to store data
    wf = wave.open('received_audio.wav', 'wb')
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
                check_follow_true(client_socket, float(x), float(y))
            else:
                if data:
                    # Check audio levels
                    if len(data) == 0:
                        print("Invalid data in int_data. Skipping RMS calculation.")
                        rms = 500  # or handle accordingly
                    else:
                        rms = calculate_rms(data)
                        print(f"Current RMS Level: {rms}")

                    # Reset silence timer as long as data is being received
                    silence_duration = 0
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
                        AUDIO_FILE = "received_audio.wav"
                        transcribed_text = transcription_model.transcribe(AUDIO_FILE)
                        print(f"Transcription: " + transcribed_text["text"])
                        if transcribed_text["text"] == "":
                            print("No text")
                        else:
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
                    print("No data received.")
                    silence_duration = time.time() - start_time

                    # Check if silence has been long enough to consider end of speech
                    if silence_duration >= SILENCE_TIMEOUT:
                        print("Silence detected, processing audio...")
                        wf.close()
                        
                        # Process the audio file
                        AUDIO_FILE = "received_audio.wav"
                        transcribed_text = transcription_model.transcribe(AUDIO_FILE)
                        print(f"Transcription: " + transcribed_text["text"])

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
    engine = CoquiEngine() 
    stream = TextToAudioStream(engine)
    start_server()
