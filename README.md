# Embedded IoT Project

## Overview
This project implements an IoT solution using ESP8266/ESP32 microcontrollers connected to Firebase services. It enables real-time data monitoring, storage, and control capabilities through Firebase Realtime Database, Firestore, and Storage services.

## Features
- Real-time data synchronization with Firebase Realtime Database
- Document-based data storage with Firestore
- File storage capabilities using Firebase Storage
- Secure authentication with Firebase
- Support for both ESP8266 and ESP32 platforms

## Prerequisites
- Arduino IDE or PlatformIO
- ESP8266 or ESP32 development board
- Firebase account with a project created
- Basic understanding of IoT and Arduino programming

## Installation

### Hardware Setup
1. Connect your ESP8266 or ESP32 board to your computer
2. Prepare any sensors or components according to your specific implementation

### Software Setup
1. Install PlatformIO or Arduino IDE
   - For PlatformIO: Follow the [PlatformIO installation guide](https://docs.platformio.org/en/latest/installation.html)
   - For Arduino IDE: Download from [Arduino website](https://www.arduino.cc/en/software)
3. Install required libraries:
   - Firebase Arduino Client Library for ESP8266 and ESP32 (version 4.4.17 or compatible)
   - ESP32 Servo library

### Firebase Setup
1. Create a new Firebase project at [Firebase Console](https://console.firebase.google.com/)
2. Set up required Firebase services (Realtime Database, Firestore, Storage, etc.)
3. Configure authentication and generate API keys
4. Update your project credentials in the code

## Usage

### Create Configuration File (`config.h`)

```cpp
#ifndef CONFIG_H
#define CONFIG_H

const char* USER_EMAIL = "xxx";
const char* USER_PASSWORD = "xxx";
const char* API_KEY = "xxx";
const char* DATABASE_URL = "xxx";
#endif
```

## Project Structure
- `/src` - Source code files
- `/lib` - External libraries
- `/examples` - Example implementations
- `/docs` - Documentation

## Troubleshooting
- Ensure correct WiFi and Firebase credentials
- Check serial monitor for detailed error messages
- Verify Firebase rules to allow read/write operations
- Ensure adequate power supply for the ESP8266/ESP32 board

## Contributing
Contributions to this project are welcome. Please feel free to submit a Pull Request.

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments
- [Firebase Arduino Client Library for ESP8266 and ESP32](https://github.com/mobizt/Firebase-ESP-Client)
- [ESP8266 Community](https://github.com/esp8266/Arduino)
- [ESP32 Development Team](https://github.com/espressif/arduino-esp32)

## Contact
- Author: Nguyen Duc Duy
- Email: nguyenducduypc160903@gmail.com
