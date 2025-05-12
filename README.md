# Raspberry Pi 5 Advanced Security System

This security system combines motion sensors with AI facial recognition and RFID authentication for enhanced home security. When motion is detected, the system captures photos, identifies faces, and only requires RFID authentication for unknown faces. 

## Features

- **Dual Authentication:** Facial recognition + RFID
- **Automatic Photo Capture:** Takes photos when motion is detected
- **Smart Authentication:** Only requires RFID for unknown faces
- **Visual Indicators:** Uses LED lights to show system status
- **Logging:** Saves annotated photos of detected faces

## Hardware Requirements

- Raspberry Pi 5 (or Pi 4/3B+)
- PIR Motion Sensor
- MFRC522 RFID Reader + Tags
- Picamera2 compatible camera module
- Red and Green LEDs
- Jumper wires
- Breadboard

## GPIO Connections

| Component | GPIO Pin (BCM) |
|-----------|---------------|
| PIR Motion Sensor | 18 |
| Green LED | 27 |
| Red LED | 17 |
| RFID Reader | SPI Pins (Check MFRC522 library docs) |

## Installation

### 1. Set Up Your Environment

Begin by setting up a Python virtual environment for your project. This keeps dependencies organized and avoids conflicts.

```bash
# Update your Pi
sudo apt update
sudo apt upgrade -y

# Install required system packages
sudo apt install -y python3-venv python3-dev python3-pip libatlas-base-dev libhdf5-dev libhdf5-serial-dev libopenjp2-7 libtiff5 libjpeg62-turbo-dev

# Create a virtual environment
python3 -m venv security_env

# Activate the environment
source security_env/bin/activate
```

### 2. Install Dependencies

```bash
# Install required Python packages
pip install face_recognition opencv-python numpy picamera2 RPi.GPIO

# Install lgpio support
sudo apt install -y python3-lgpio
```

### 3. Setting Up Face Recognition

This project uses OpenCV and facial recognition. For detailed setup instructions and best practices, follow this excellent guide: [Face Recognition with Raspberry Pi and OpenCV](https://core-electronics.com.au/guides/face-recognition-with-raspberry-pi-and-opencv/#setting-up-virtual-environment). After following the guide, ensure you install all dependencies in your virtual environment and copy the integrated_security_system.py code into your project folder.


### 4. Setting Up RFID

1. Install the MFRC522 library:

```bash
pip install mfrc522
```

2. Configure your RFID tags:
   - Open the integrated_security_system.py file
   - Locate the `authorized_rfid_tags` list
   - Replace the example UIDs with your actual RFID tag UIDs

### 5. Run the Security System

```bash
# Make sure your virtual environment is activated
source security_env/bin/activate

# Run the security system
python integrated_security_system.py
```

## Operation Instructions

1. **System Startup:**
   - The system starts in ARMED mode
   - Both LEDs will blink briefly during initialization

2. **Motion Detection:**
   - When motion is detected, the system captures a photo
   - If a recognized face is detected, system remains armed
   - If an unknown face is detected, system enters GRACE period

3. **GRACE Period:**
   - The green LED blinks
   - You have 20 seconds to scan an authorized RFID tag
   - If no tag is scanned, the system enters ALARM state

4. **ALARM State:**
   - The red LED blinks rapidly
   - System returns to ARMED state after alarm sequence

5. **Manual Disarming:**
   - Scan an authorized RFID tag at any time to disarm the system
   - The green LED will light up when disarmed
   - Scan again to rearm

## Customization

### Changing Authentication Time:

Edit the following variables in the integrated_security_system.py file:
```python
GRACE_PERIOD_SECONDS = 20  # Change to adjust authentication time
RFID_BUFFER_SECONDS = 5    # Change to adjust time between RFID scans
```

### Adding More Authorized Users:

1. Create folders for new users in the 'dataset' directory
2. Add face images for each new user
3. Re-run the encode_faces.py script to update encodings

### Adding RFID Tags:

Update the authorized_rfid_tags list with new tag UIDs:
```python
authorized_rfid_tags = [
    "ABCDEF12",  # Example tag
    "12345678",
    "YOUR_NEW_TAG_UID"  # Add new tags here
]
```

## Troubleshooting

### No Faces Detected:
- Check camera angle and lighting
