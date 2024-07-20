import cv2
import numpy as np
import socket
import threading
import time

# Define the paths to the model files
weights_file_abs_path = r"C:/Users/DELL/Desktop/RealTimeDataProcessing/yolov3.weights"
config_file_abs_path = r"C:/Users/DELL/Desktop/RealTimeDataProcessing/yolov3.cfg"
names_file_abs_path = r"C:/Users/DELL/Desktop/RealTimeDataProcessing/coco.names"

# Load YOLO
net = cv2.dnn.readNet(weights_file_abs_path, config_file_abs_path)
# Uncomment the following lines if you have a compatible GPU
# net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
# net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]

# Load class names
with open(names_file_abs_path, 'r') as f:
    classes = [line.strip() for line in f.readlines()]

# URL of the video stream
URL = "http://192.168.1.88/stream"

# Open the video stream
cap = cv2.VideoCapture(URL)

# Check if the video stream was opened successfully
if not cap.isOpened():
    print(f"Error: Unable to open video stream at {URL}")
    exit()

# Replace with your ESP32's IP address and port
esp32_ip = "192.168.1.88"
esp32_port = 8080

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the ESP32 WiFi server
try:
    client_socket.connect((esp32_ip, esp32_port))
    client_socket.settimeout(5.0)  # Increase the timeout for socket operations
except socket.error as e:
    print(f"Socket error: {e}")
    exit()

# Shared frame buffer
frame_buffer = None
frame_lock = threading.Lock()

def send_command(command):
    try:
        print(f"Sending command: {command}")
        client_socket.send(command.encode())
        # Receive response from ESP32 (if any)
        response = client_socket.recv(1024)
        print("Response from ESP32:", response.decode())
    except socket.timeout:
        print("Warning: Socket operation timed out")
    except socket.error as e:
        print(f"Socket error: {e}")

def capture_frames():
    global frame_buffer
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to read frame from video stream")
            break
        with frame_lock:
            frame_buffer = frame.copy()
        time.sleep(0.01)  # Small delay to reduce CPU usage

def process_frames():
    global frame_buffer
    while True:
        with frame_lock:
            if frame_buffer is None:
                continue
            frame = frame_buffer.copy()
        
        # Downscale the frame for faster processing
        frame = cv2.resize(frame, (416, 416))

        # Get the height, width, and channels of the frame
        height, width, channels = frame.shape

        # Detecting objects
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Showing information on the screen
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = confidences[i]
                color = (0, 255, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                command = "LEFT 1000 1000 1000 1000"  # send command with speed value 50
                threading.Thread(target=send_command, args=(command,)).start()

        # Display the output image
        cv2.imshow('Object Detection', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Start the threads
capture_thread = threading.Thread(target=capture_frames)
process_thread = threading.Thread(target=process_frames)

capture_thread.start()
process_thread.start()

capture_thread.join()
process_thread.join()

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
client_socket.close()