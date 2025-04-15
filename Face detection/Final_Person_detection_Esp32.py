import cv2
import socket
import time
import os
import json
from collections import defaultdict

# Config paths for object detection
configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'
classFile = 'coco.names'

# Load class names
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

# Load the DNN model
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Thresholds for object size classification
BIG_BOX_THRESHOLD = 20000
MEDIUM_BOX_THRESHOLD = 4750

# ESP32 configuration
esp32_ip = "192.168.96.226"  # ESP32's IP address
esp32_port = 4210  # ESP32's UDP port

# Python server configuration
python_server_ip = "0.0.0.0"  # Listen on all available interfaces
python_server_port = 4230  # Port that ESP32 sends to

# State tracking
active_lights = defaultdict(bool)  # Track which lights are currently active
last_detection_time = defaultdict(float)  # Track when each light was last detected
DETECTION_TIMEOUT = 5.0  # Time in seconds before considering a detection stale

# Function to send message to ESP32
def send_to_esp32(message):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(2)
            sock.sendto(message.encode(), (esp32_ip, esp32_port))
            print(f"Message '{message}' sent to ESP32.")
    except Exception as e:
        print(f"Failed to send message to ESP32: {e}")

# Camera URLs
cam_urls = ['http://192.168.96.78/cam-hi.jpg', 'http://192.168.96.49/cam-hi.jpg']  # ESP32 Camera URLs

# Setup UDP server to listen for requests from ESP32
def setup_udp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((python_server_ip, python_server_port))
    server_socket.setblocking(False)  # Non-blocking socket
    print(f"Python server listening on UDP port {python_server_port}")
    return server_socket

# Initialize UDP server
udp_server = setup_udp_server()

request_light_active = False
last_status_update = 0

while True:
    current_time = time.time()
    
    # Check for UDP messages from ESP32
    try:
        data, addr = udp_server.recvfrom(1024)
        message = data.decode('utf-8')
        print(f"Received message from ESP32 {addr}: {message}")
        
        if message == "REQUEST_LIGHT_STATUS":
            # ESP32 is requesting the current status of all lights
            request_light_active = True
            print("REQUEST_LIGHT_STATUS received from ESP32. Processing cameras...")
    except BlockingIOError:
        # No data available, continue with the loop
        pass

    # Only process cameras if we have an active request or it's time for a periodic update
    if request_light_active or (current_time - last_status_update > 1.0):  # Update every second
        # Process camera feeds
        current_detections = set()  # Track detections in this frame
        
        for i, url in enumerate(cam_urls):
            cap = cv2.VideoCapture(url)
            ret, img = cap.read()
            if not ret:
                print(f"Failed to capture image from Camera {i+1}. Skipping...")
                cap.release()
                continue

            # Rotate image if necessary
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

            # Object detection
            classIds, confs, bbox = net.detect(img, confThreshold=0.5)
            
            if len(classIds) != 0:
                # Track the largest person for each camera zone
                camera_zones = {}  # camera_zones[zone_id] = (size, light_id)
                
                for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    className = classNames[classId - 1]
                    if className == "person":
                        x, y, w, h = box
                        box_size = w * h
                        
                        # Calculate which zone this person is in
                        # For simplicity, let's divide the image into 4 zones (2x2 grid)
                        zone_x = 0 if x < img.shape[1]/2 else 1
                        zone_y = 0 if y < img.shape[0]/2 else 1
                        zone_id = f"cam{i}_zone{zone_x}{zone_y}"
                        
                        # Update the largest person in this zone
                        if zone_id not in camera_zones or box_size > camera_zones[zone_id][0]:
                            # Determine which light to activate based on zone and size
                            if i == 0:  # First camera
                                if box_size > BIG_BOX_THRESHOLD:
                                    light_id = 1
                                elif box_size > MEDIUM_BOX_THRESHOLD:
                                    light_id = 2
                                else:
                                    light_id = None
                            else:  # Second camera
                                if box_size > BIG_BOX_THRESHOLD:
                                    light_id = 3
                                elif box_size > MEDIUM_BOX_THRESHOLD:
                                    light_id = 4
                                else:
                                    light_id = None
                                    
                            if light_id:
                                camera_zones[zone_id] = (box_size, light_id)
                        
                        # Draw bounding box and label
                        color = (0, 255, 0)
                        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                        label = f"{className} {confidence*100:.2f}%"
                        cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Process the largest person in each zone
                for zone_id, (size, light_id) in camera_zones.items():
                    if light_id:
                        current_detections.add(light_id)
                        last_detection_time[light_id] = current_time
                        
                        # Activate this light if not already active
                        if not active_lights[light_id]:
                            active_lights[light_id] = True
                            send_to_esp32(f"light{light_id}_ON")
            
            # Display the live feed
            cv2.imshow(f'Camera {i+1}', img)
            cap.release()
        
        # Check for lights that need to be turned off
        lights_to_deactivate = []
        for light_id in range(1, 5):  # Lights 1-4
            if active_lights[light_id] and light_id not in current_detections:
                # Check if detection timeout has passed
                if current_time - last_detection_time[light_id] > DETECTION_TIMEOUT:
                    lights_to_deactivate.append(light_id)
        
        # Turn off lights that are no longer needed
        for light_id in lights_to_deactivate:
            active_lights[light_id] = False
            send_to_esp32(f"light{light_id}_OFF")
        
        # If we had an active request, send the status summary
        if request_light_active:
            # Create a status message with all active lights
            active_light_list = [f"light{light_id}" for light_id in range(1, 5) if active_lights[light_id]]
            status_message = ",".join(active_light_list) if active_light_list else "NO_LIGHTS"
            send_to_esp32(f"STATUS:{status_message}")
            request_light_active = False
            
        last_status_update = current_time

    # Break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()