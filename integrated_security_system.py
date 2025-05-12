#!/usr/bin/env python3
# -*- coding: utf8 -*-
#
# Raspberry Pi 5 Security System with Facial Recognition and RFID
# Integrates RFID reader, PIR motion sensor, camera for facial recognition, and LED indicators

import MFRC522
import lgpio
import signal
import time
import datetime
import threading
import face_recognition
import cv2
import numpy as np
import pickle
import os
from picamera2 import Picamera2

# GPIO Configuration
GPIO_CHIP = 0        # Default chip
PIR_GPIO = 18        # Motion sensor (BCM pin 18)
GREEN_LED_GPIO = 27  # Green LED for disarmed state
RED_LED_GPIO = 17    # Red LED for alarm

# RFID setup
MIFAREReader = MFRC522.MFRC522()

# System states
ARMED = "ARMED"          # System is armed and monitoring
GRACE_PERIOD = "GRACE"   # Motion detected, waiting for RFID scan
DISARMED = "DISARMED"    # System disarmed after valid RFID
ALARM = "ALARM"          # Alarm triggered after grace period with no valid RFID

# System configuration
GRACE_PERIOD_SECONDS = 20  # Time to scan RFID card after motion detection
RFID_BUFFER_SECONDS = 5    # Buffer time between RFID scans
PHOTOS_DIRECTORY = "security_photos"  # Directory to store security photos

# Global variables
current_state = ARMED
camera_in_use = False
continue_running = True
handling_motion = False
last_rfid_scan_time = 0
authorized_rfid_tags = [
    "ABCDEF12",  # Example tag - replace with your actual tag UIDs
    "12345678" 
]

# Initialize GPIO
h = lgpio.gpiochip_open(GPIO_CHIP)
lgpio.gpio_claim_input(h, PIR_GPIO)
lgpio.gpio_claim_output(h, GREEN_LED_GPIO)
lgpio.gpio_claim_output(h, RED_LED_GPIO)

# Turn off both LEDs initially
lgpio.gpio_write(h, GREEN_LED_GPIO, 0)
lgpio.gpio_write(h, RED_LED_GPIO, 0)

# Create photos directory if it doesn't exist
if not os.path.exists(PHOTOS_DIRECTORY):
    os.makedirs(PHOTOS_DIRECTORY)

# Load pre-trained face encodings
print("[INFO] Loading face encodings...")
try:
    with open("encodings.pickle", "rb") as f:
        data = pickle.loads(f.read())
    known_face_encodings = data["encodings"]
    known_face_names = data["names"]
    print(f"[INFO] Loaded {len(known_face_encodings)} face encodings for {len(set(known_face_names))} individuals")
except FileNotFoundError:
    print("[WARNING] encodings.pickle not found. Running without facial recognition.")
    known_face_encodings = []
    known_face_names = []

def uidToString(uid):
    """Convert RFID UID to string format"""
    mystring = ""
    for i in uid:
        mystring = format(i, '02X') + mystring
    return mystring

def end_program(signal, frame):
    """Handle program termination"""
    global continue_running
    print("\nProgram terminated. Cleaning up...")
    continue_running = False
    # Turn off LEDs
    lgpio.gpio_write(h, GREEN_LED_GPIO, 0)
    lgpio.gpio_write(h, RED_LED_GPIO, 0)
    # Close GPIO
    lgpio.gpiochip_close(h)

def take_photo(filename=None):
    """Take a photo and return the image array and filename"""
    global camera_in_use
    
    if camera_in_use:
        print("Camera already in use")
        return None, None
    
    camera_in_use = True
    img = None
    
    try:
        # Initialize camera
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)}))
        picam2.start()
        time.sleep(1)  # Allow camera to initialize
        
        # Capture image
        img = picam2.capture_array()
        
        # Generate filename if not provided
        if filename is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{PHOTOS_DIRECTORY}/security_{timestamp}.jpg"
        
        # Save the image
        cv2.imwrite(filename, img)
        print(f"Photo saved as '{filename}'")
        
        # Stop camera
        picam2.stop()
        
    except Exception as e:
        print(f"Camera error: {str(e)}")
        filename = None
    
    finally:
        camera_in_use = False
    
    return img, filename

def recognize_faces(image):
    """Recognize faces in the image and return results"""
    # If no encodings loaded, return no matches
    if not known_face_encodings:
        return [], [], False
    
    # Process the image for face recognition
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # Find all face locations and encodings
    face_locations = face_recognition.face_locations(rgb_image)
    face_encodings = face_recognition.face_encodings(rgb_image, face_locations)
    
    face_names = []
    recognized = False
    
    # Process each face found
    for face_encoding in face_encodings:
        # Compare with known faces
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        name = "Unknown"
        
        # Find the best match
        if True in matches:
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]
                recognized = True
        
        face_names.append(name)
    
    # Draw boxes and names on the image for logging purposes
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Draw a box around the face
        cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)
        
        # Draw label with name
        cv2.rectangle(image, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
    
    return face_locations, face_names, recognized

def blink_led(gpio, count=10, interval=0.2):
    """Blink LED specified number of times"""
    for i in range(count):
        if not continue_running:
            break
        lgpio.gpio_write(h, gpio, 1)
        time.sleep(interval)
        lgpio.gpio_write(h, gpio, 0)
        time.sleep(interval)

def check_rfid():
    """Check if RFID card is present and authorized"""
    global last_rfid_scan_time
    
    # Check if enough time has passed since last RFID scan
    current_time = time.time()
    if current_time - last_rfid_scan_time < RFID_BUFFER_SECONDS:
        return False
    
    # Scan for cards
    (status, TagType) = MIFAREReader.MFRC522_Request(MIFAREReader.PICC_REQIDL)
    
    # If a card is found
    if status == MIFAREReader.MI_OK:
        # Get the UID of the card
        (status, uid) = MIFAREReader.MFRC522_SelectTagSN()
        
        # If we have the UID, check if it's authorized
        if status == MIFAREReader.MI_OK:
            card_uid = uidToString(uid)
            print(f"Card detected: {card_uid}")
            
            if card_uid in authorized_rfid_tags:
                print("Authorized card detected!")
                last_rfid_scan_time = current_time
                return True
            else:
                print("Unauthorized card!")
                return False
    
    return False

def toggle_system_state():
    """Toggle between armed and disarmed states"""
    global current_state
    
    if current_state == ARMED or current_state == GRACE_PERIOD:
        # Disarm the system
        current_state = DISARMED
        print("System DISARMED. Scan card again to re-arm.")
        # Turn on green LED
        lgpio.gpio_write(h, GREEN_LED_GPIO, 1)
        lgpio.gpio_write(h, RED_LED_GPIO, 0)
    
    elif current_state == DISARMED:
        # Re-arm the system
        current_state = ARMED
        print("System ARMED")
        # Turn off green LED
        lgpio.gpio_write(h, GREEN_LED_GPIO, 0)
        
        # Confirmation blink pattern
        blink_led(GREEN_LED_GPIO, count=2, interval=0.2)

def transition_to_alarm():
    """Transition system to alarm state"""
    global current_state, handling_motion
    
    current_state = ALARM
    print("ALARM TRIGGERED!")
    
    # Flash red LED
    blink_led(RED_LED_GPIO, count=20, interval=0.25)
    
    # Return to armed state
    lgpio.gpio_write(h, RED_LED_GPIO, 0)
    current_state = ARMED
    print("System ARMED")
    
    # Reset motion handling flag
    handling_motion = False

def handle_motion_detection():
    """Handle motion detection event with facial recognition"""
    global current_state, handling_motion
    
    if current_state == ARMED and not handling_motion:
        handling_motion = True
        print("Motion detected! Taking photo for facial recognition...")
        
        # Take a photo
        image, filename = take_photo()
        
        if image is not None:
            # Process the image for facial recognition
            face_locations, face_names, recognized = recognize_faces(image)
            
            # Save annotated image with face boxes
            if face_locations:
                annotated_filename = filename.replace(".jpg", "_annotated.jpg")
                cv2.imwrite(annotated_filename, image)
                
                print(f"Detected {len(face_locations)} faces:")
                for name in face_names:
                    print(f"- {name}")
                
                if recognized:
                    print("Authorized person(s) recognized.")
                    # Disable alarm but keep in armed state
                    handling_motion = False
                    return
                else:
                    print("Unknown person detected. Starting grace period for RFID authentication.")
                    current_state = GRACE_PERIOD
                    
                    # Enter grace period for RFID authentication
                    grace_end_time = time.time() + GRACE_PERIOD_SECONDS
                    authorized = False
                    
                    # Blink green LED during grace period
                    threading.Thread(target=blink_led, args=(GREEN_LED_GPIO, 40, 0.5), daemon=True).start()
                    
                    while time.time() < grace_end_time and continue_running and not authorized and current_state == GRACE_PERIOD:
                        # Check for RFID card
                        if check_rfid():
                            authorized = True
                            break
                        
                        time.sleep(0.1)
                    
                    # If authorized card detected, disarm system
                    if authorized:
                        toggle_system_state()
                    # Otherwise trigger alarm (only if still in GRACE_PERIOD)
                    elif continue_running and current_state == GRACE_PERIOD:
                        transition_to_alarm()
            else:
                print("No faces detected in image.")
                handling_motion = False
        else:
            print("Failed to capture image.")
            handling_motion = False

# Register signal handler for cleanup
signal.signal(signal.SIGINT, end_program)

# Main program loop
def main():
    global handling_motion
    
    print("Security System Starting...")
    print("System ARMED")
    
    # Check if facial recognition is enabled
    if not known_face_encodings:
        print("WARNING: No face encodings loaded. System will rely solely on RFID authentication.")
    else:
        print(f"Facial recognition enabled with {len(set(known_face_names))} authorized individuals.")
    
    # Initialization blink pattern to show system is ready
    blink_led(GREEN_LED_GPIO, count=3, interval=0.2)
    blink_led(RED_LED_GPIO, count=3, interval=0.2)
    
    motion_prev_state = 0
    
    while continue_running:
        # Check for motion
        motion_current_state = lgpio.gpio_read(h, PIR_GPIO)
        
        # Motion detected (rising edge)
        if motion_current_state == 1 and motion_prev_state == 0:
            print("Motion detected!")
            if current_state == ARMED and not handling_motion:
                threading.Thread(target=handle_motion_detection, daemon=True).start()
        
        motion_prev_state = motion_current_state
        
        # Check for RFID to toggle system state
        if check_rfid():
            toggle_system_state()
            # If we just disarmed via RFID, reset motion handling flag
            if current_state == DISARMED:
                handling_motion = False
        
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Ensure clean exit
        lgpio.gpio_write(h, GREEN_LED_GPIO, 0)
        lgpio.gpio_write(h, RED_LED_GPIO, 0)
        lgpio.gpiochip_close(h)