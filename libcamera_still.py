import os
import subprocess
import time
import cv2
from ultralytics import YOLO

# Load YOLO model
model = YOLO("yolov10n.pt")
class_names_list = None
with open('coco.names', 'r') as f:
    class_names_list = [line.strip() for line in f.readlines()]

# Function to run YOLO on an image
def run_yolo_on_image(image_path):
    try:
        frame = cv2.imread(image_path)
        if frame is None:
            print(f"Failed to read image {image_path}")
            return

        # Resize frame if needed (optional, YOLO will auto-resize internally)
        # resized_frame = cv2.resize(frame, (640, 480))

        # YOLO inference
        results = model(frame)[0]
        for result in results.boxes.data:
            x_min, y_min, x_max, y_max, confidence, class_id = result.tolist()
            if int(class_id) < len(class_names_list):
                label = class_names_list[int(class_id)]
                print(f"Detected {label} with confidence {confidence:.2f} at [{x_min}, {y_min}, {x_max}, {y_max}]")

    except Exception as e:
        print(f"Error during YOLO inference: {e}")

# Function to capture images using libcamera-still
def capture_images(duration=5, fps=5, output_dir="captured_images"):
    os.makedirs(output_dir, exist_ok=True)  # Create the output directory if it doesn't exist
    interval = 1.0 / fps  # Time between captures in seconds
    start_time = time.time()

    while time.time() - start_time < duration:
        timestamp = int(time.time() * 1000)  # Unique timestamp for filename
        image_path = os.path.join(output_dir, f"frame_{timestamp}.jpg")
        
        # Capture the image using libcamera-still
        cmd = ["libcamera-still", "-n", "-o", image_path]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print(f"Captured image: {image_path}")
        run_yolo_on_image(image_path)  # Run YOLO inference on the captured image

        time.sleep(interval)  # Maintain consistent FPS

    print("Image capture complete.")

# Main function
def main():
    try:
        duration = 5  # Duration of capture in seconds
        fps = 5       # Frames per second

        print(f"Starting image capture at {fps} FPS for {duration} seconds...")
        capture_images(duration=duration, fps=fps)

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting.")
    except Exception as e:
        print(f"Error in main: {e}")

if __name__ == "__main__":
    main()
