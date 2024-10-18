import socket
import wave
import pyaudio
import os
from TTS.api import TTS
import torch
from gpt4all import GPT4All
import whisper

#TODO
# Add memory for TTS

# Configure the server
HOST = '0.0.0.0' 
PORT = 8080     

# Audio settings
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000 

device = "cuda" if torch.cuda.is_available() else "cpu"

transcription_model = whisper.load_model("base").to("cuda")

# Initialize TTS (Assuming Coqui TTS is already set up)
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to(device)

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
        response = model.generate(user_input, max_tokens=450, temp=1.1, top_k = 60, top_p = 0.85, min_p = 0.045, repeat_penalty = 2.1, n_batch=16)
        response_automated = f"{response}"
        return response_automated
    
# Function to generate TTS and send back
def generate_and_send_tts(client_socket, text):
    tts_text = generate_response(text)
    print(f"Generating TTS for: {tts_text}")
    
    # Generate TTS audio (you can also customize sampling rate here)
    wav_bytes = tts.tts_to_file(text=tts_text, speaker_wav=["emily1.wav", "IMG_1306.wav", "IMG_1307.wav", "IMG_1308.wav","IMG_1309.wav","IMG_1310.wav","IMG_1313.wav","IMG_1314.wav","IMG_1315.wav"], language="en", file_path="balsshd.wav")

    # Open the wav file and send it back
    wf = wave.open('response.wav', 'rb')
    
    while True:
        data = wf.readframes(CHUNK_SIZE)
        if not data:
            break
        client_socket.sendall(data)
    
    wf.close()
    os.remove('response.wav')  # Clean up the file

def handle_client(client_socket):
    print("Client connected")

    # Open a file to store the incoming audio
    wf = wave.open('received_audio.wav', 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(pyaudio.get_sample_size(FORMAT))
    wf.setframerate(RATE)

    try:
        while True:
            data = client_socket.recv(CHUNK_SIZE)
            if not data:
                break  # No more data, client closed connection

            wf.writeframes(data)
        
        print("Audio received, passing it through LLM...")
        wf.close()

        # Example: Pass the received audio through an LLM (This is where you'd use transcription)
        AUDIO_FILE = "received_audio.wav"
        transcribed_text = transcription_model.transcribe(AUDIO_FILE)  # Replace with real transcription logic

        # Send back a TTS response
        generate_and_send_tts(client_socket, transcribed_text)

    finally:
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
