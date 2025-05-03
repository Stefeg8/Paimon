import subprocess
import cv2
import numpy as np
from ultralytics import YOLO

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

model = YOLO("inc/yolov10n.pt") 
class_names_list = None
with open('coco_test.names', 'r') as f:  
    class_names_list = [line.strip() for line in f.readlines()]

# Function to read YUV420 frames and convert to BGR
def read_frame():
    yuv_size = WIDTH * HEIGHT * 3 // 2
    yuv_frame = process.stdout.read(yuv_size)
    if len(yuv_frame) != yuv_size:
        return None
    yuv_np = np.frombuffer(yuv_frame, dtype=np.uint8).reshape((HEIGHT * 3 // 2, WIDTH))
    bgr = cv2.cvtColor(yuv_np, cv2.COLOR_YUV2BGR_I420)
    return bgr

def pipeline(frame):
    results = model(frame)[0]
    for result in results.boxes.data:
        x_min, y_min, x_max, y_max, confidence, class_id = result.tolist()
        if int(class_id) < len(class_names_list) and class_names_list[int(class_id)] == "person":
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
            print(center_x,center_y)

while True:
    frame = read_frame()
    if frame is None:
        break

    # TODO: YOLOv10 detection here
    pipeline(frame)
    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
process.terminate()
