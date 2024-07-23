import cv2
import numpy as np
import socket
import time
from ultralytics import YOLO

# Load YOLO
model = YOLO("yolov8n.pt")  # Load YOLOv8 weights

# URL of the video stream
URL = "http://192.168.1.88/stream"  # Replace with your ESP32 camera stream URL

# Open the video stream
cap = cv2.VideoCapture(URL)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

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
    client_socket.settimeout(10)  # Increase the timeout for socket operations
except socket.error as e:
    print(f"Socket error: {e}")
    exit()

# Main loop for capturing and processing frames
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read frame from video stream")
        break

    # Resize frame for faster processing
    frame_resized = cv2.resize(frame, (640, 640))

    # Perform detection
    results = model(frame_resized)

    # Process results
    person_detected = False
    for result in results:
        boxes = result.boxes
        for box in boxes:
            cls = int(box.cls[0])
            if model.names[cls] == 'person':
                person_detected = True
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()  # Convert to int
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, 'Person', (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                break
        if person_detected:
            break

    # Send commands based on detection
    if person_detected:
        command = "FORWARD 100 100 100 100"  # Adjust velocity as needed
        client_socket.send((command + '\n').encode())
    else:
        # Stop the car and turn
        stop_command = "STOP 0 0 0 0"
        client_socket.send((stop_command + '\n').encode())
        time.sleep(5)  # Wait for 5 seconds before turning
        turn_command = "TURN 100 100 -100 -100"
        client_socket.send((turn_command + '\n').encode())
        time.sleep(0.5)  # Turn for 0.5 seconds
        client_socket.send((stop_command + '\n').encode())
        time.sleep(5)  # Wait for another 5 seconds before next action

    # Display the output image
    cv2.imshow('Object Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
client_socket.close()