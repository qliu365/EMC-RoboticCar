import socket

# Replace with your ESP32's IP address and port
esp32_ip = "10.94.6.115"
esp32_port = 8080

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the ESP32 WiFi server
client_socket.connect((esp32_ip, esp32_port))

# Send a command to the ESP32
command = "LEFT 0 0 0 0"  # send command with speed value 50
client_socket.send(command.encode())

# Receive response from ESP32 (if any)
response = client_socket.recv(1024)
print("Response from ESP32:", response.decode())

# Close the socket
client_socket.close()