import sounddevice as sd
from scipy.io.wavfile import write

# Configuration
FS = 44100          # Sample rate (Hz)
DURATION = 5        # Recording duration (seconds)
CHANNELS = 1        # 1 = mono, 2 = stereo
FILENAME = "output.wav"  # Output file name

print("Recording started...")
audio = sd.rec(int(DURATION * FS), samplerate=FS, channels=CHANNELS, dtype='int16')
sd.wait()  # Wait until recording is finished
print("Recording finished. Saving to file...")

write(FILENAME, FS, audio)
print(f"Audio saved to {FILENAME}")
