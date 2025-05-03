import subprocess
import cv2
import numpy as np

# Set resolution
WIDTH = 1280
HEIGHT = 720

# Start libcamera-vid as a subprocess
cmd = [
    'libcamera-vid',
    '--width', str(WIDTH),
    '--height', str(HEIGHT),
    '--codec', 'yuv420',
    '--nopreview',
    '-t', '0',
    '-o', '-'
]
process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=WIDTH * HEIGHT * 3 // 2)

# Function to read YUV420 frames and convert to BGR
def read_frame():
    yuv_size = WIDTH * HEIGHT * 3 // 2
    yuv_frame = process.stdout.read(yuv_size)
    if len(yuv_frame) != yuv_size:
        return None
    yuv_np = np.frombuffer(yuv_frame, dtype=np.uint8).reshape((HEIGHT * 3 // 2, WIDTH))
    bgr = cv2.cvtColor(yuv_np, cv2.COLOR_YUV2BGR_I420)
    return bgr

while True:
    frame = read_frame()
    if frame is None:
        break

    # TODO: YOLOv10 detection here
    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
process.terminate()
