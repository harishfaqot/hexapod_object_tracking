import cv2
import time
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
    "darknet/cfg/yolov4-tiny.cfg",
    "darknet/cfg/coco.data",
    "models/yolov4-tiny.weights",
    batch_size=1
)

# Input size expected by YOLO
width = darknet.network_width(network)
height = darknet.network_height(network)

# Open CSI camera
cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

darknet_image = darknet.make_image(width, height, 3)

while True:
    start_time = time.time()

    ret, frame = cap.read()
    if not ret:
        break

    # Preprocess
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))
    darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())

    # Detection
    detections = darknet.detect_image(network, class_names, darknet_image)
    image = darknet.draw_boxes(detections, frame_resized, class_colors)

    # FPS Calculation
    fps = 1.0 / (time.time() - start_time)
    cv2.putText(image, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Show image
    image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imshow("YOLOv4-Tiny", image_bgr)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
