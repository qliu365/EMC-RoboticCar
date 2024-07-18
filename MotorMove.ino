#include <WiFi.h>
#include <WiFiServer.h>
#include "Freenove_4WD_Car_For_ESP32.h"

const char* ssid = "SpectrumSetup-DB";
const char* password = "epicnature840";
WiFiServer server(8080);

void setup() {
    PCA9685_Setup(); // Initialize PCA9685 for motor control
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        Serial.println("Client connected");
        while (client.connected()) {
            if (client.available()) {
                String command = client.readStringUntil('\n');
                command.trim(); // Remove any whitespace
                Serial.println("Received command: " + command);
                processCommand(command);
            }
        }
        client.stop();
        Serial.println("Client disconnected");
    }
}

void processCommand(const String& command) {
    int speeds[4] = {0, 0, 0, 0}; // Array to hold the four speed values
    int spaceIndex = 0;
    int prevIndex = 0;
    int count = 0;

    // Split the command string by spaces
    for (int i = 0; i < 5; i++) {
        spaceIndex = command.indexOf(' ', prevIndex);
        if (spaceIndex == -1) { // If no more spaces, take the rest of the string
            if (i == 0) {
                // Extract the command
                String cmd = command.substring(prevIndex);
                Serial.print("Command: "); Serial.println(cmd);
            } else {
                speeds[i-1] = command.substring(prevIndex).toInt();
            }
            count++;
            break;
        } else {
            if (i == 0) {
                // Extract the command
                String cmd = command.substring(prevIndex, spaceIndex);
                Serial.print("Command: "); Serial.println(cmd);
            } else {
                speeds[i-1] = command.substring(prevIndex, spaceIndex).toInt();
            }
            prevIndex = spaceIndex + 1;
            count++;
        }
    }

    // Check if the command has the correct number of parts
    if (count == 5) { // 5 parts: command + 4 speeds
        String cmd = command.substring(0, command.indexOf(' ')); // Extract the command
        Serial.print("Command: "); Serial.println(cmd);
        Serial.print("Speeds: ");
        for (int i = 0; i < 4; i++) {
            Serial.println(speeds[i]);
        }

        // Execute the command using Motor_Move
        if (cmd == "FORWARD") {
            Serial.println("Moving forward");
            Motor_Move(speeds[0], speeds[1], speeds[2], speeds[3]);
        } else if (cmd == "BACKWARD") {
            Serial.println("Moving backward");
            Motor_Move(-speeds[0], -speeds[1], -speeds[2], -speeds[3]);
        } else if (cmd == "LEFT") {
            Serial.println("Turning left");
            Motor_Move(-speeds[0], -speeds[1], speeds[2], speeds[3]);
        } else if (cmd == "RIGHT") {
            Serial.println("Turning right");
            Motor_Move(speeds[0], speeds[1], -speeds[2], -speeds[3]);
        } else if (cmd == "STOP") {
            Serial.println("Stopping");
            Motor_Move(0, 0, 0, 0);
        } else {
            Serial.println("Unknown command");
        }
    } else {
        Serial.println("Invalid command format");
    }
}