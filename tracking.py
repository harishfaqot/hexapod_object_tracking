import cv2
import time
import numpy as np
from darknet import darknet

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )

# Load YOLOv4-tiny
network, class_names, class_colors = darknet.load_network(
    "models/yolov4-tiny-custom.cfg",
    "models/obj.data",
    "models/yolov4-tiny-custom_best.weights",
    batch_size=1
)

# Get expected input size for the network
width = darknet.network_width(network)
height = darknet.network_height(network)
print(width, height)

# Open the camera
cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

darknet_image = darknet.make_image(width, height, 3)

rel_x = 0
rel_y = 0
obj_label = None

def track_object():
    global rel_x, rel_y, obj_label
    
    start_time = time.time()

    ret, frame = cap.read()
    frame = cv2.resize(frame, (300, 200))

    # Get center of the frame
    center_x = frame.shape[1] // 2
    center_y = frame.shape[0] // 2

    # Preprocess image
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))
    darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())

    # Run detection
    detections = darknet.detect_image(network, class_names, darknet_image)

    # Scale ratios
    scale_x = frame.shape[1] / width
    scale_y = frame.shape[0] / height

    rel_x = 0
    rel_y = 0
    obj_label = None
    area = -1

    for label, confidence, bbox in detections:
        obj_label = label

        x, y, w, h = bbox
        x = int(x * scale_x)
        y = int(y * scale_y)
        w = int(w * scale_x)
        h = int(h * scale_y)

        # Top-left corner
        x1 = int(x - w / 2)
        y1 = int(y - h / 2)

        # Object center
        obj_x = x
        obj_y = y

        # Relative position
        rel_x = obj_x - center_x
        rel_y = -(obj_y - center_y)
        position = "Left" if rel_x < 0 else "Right"

        # Draw bounding box and info
        area = w * h
        cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
        cv2.circle(frame, (obj_x, obj_y), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"{label}: {float(confidence):.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if label != "korban":
            obj_label = None
            continue

        cv2.line(frame, (center_x, center_y), (obj_x, obj_y), (255, 0, 0), 2)
        cv2.putText(frame, f"Rel X: {rel_x}, Rel Y: {rel_y} | {position}", (x1, y1 + h + 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Print info
        print(f"{label} | X: {rel_x}, Y: {rel_y} | Position: {position}")

    # Draw center point
    cv2.circle(frame, (center_x, center_y), 5, (255, 255, 255), -1)

    # Show frame
    fps = 1.0 / (time.time() - start_time)
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    return frame, rel_x, rel_y, obj_label, area

if __name__ == '__main__':
    while True:
        frame, rel_x, rel_y, obj_label, area = track_object()
        print(f"vrot = {rel_x / 250:.2f} {obj_label} {area}")
        cv2.imshow("YOLOv4-Tiny - Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
