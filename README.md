# ESP32 Room Scanner

A small sensor scanner built around an ESP32.

The servo sweeps an HC-SR04 ultrasonic sensor from 0° to 180° and sends the measured distance together with IMU, GPS and temperature data to a PC over USB.

The PC runs a Python visualization that displays the collected scan data.

## Hardware

- ESP32
- HC-SR04
- MG996R servo
- MPU6050
- GPS module
- DHT11
- Voltage divider for HC-SR04 Echo

## How it works

1. The servo moves the ultrasonic sensor.
2. The HC-SR04 measures the distance.
3. The MPU6050 provides orientation data.
4. GPS provides location when available.
5. DHT11 measures temperature.
6. ESP32 sends everything as JSON over Serial.
7. Python reads the serial data and visualizes the scan.

## Data

The ESP32 sends data in this format:

```json
{
  "angle": 90,
  "dist": 1250,
  "pitch": 1.25,
  "roll": -0.42,
  "yaw": 12.30,
  "temp": 24.0,
  "lat": 43.3209,
  "lon": 21.8958
}
```

Distance is given in millimeters.

## Project Structure

```text
ESP32-Room-Scanner/
├── ESP32_Room_Scanner/
│   └── ESP32_Room_Scanner.ino
├── visualization/
│   └── visualization.py
└── README.md
```

## Libraries

- ESP32Servo
- TinyGPSPlus
- DHT sensor library
- MPU6050_light
- Wire

## Notes

The HC-SR04 Echo pin should not be connected directly to a 3.3V ESP32 GPIO. Use a voltage divider.

The GPS coordinates will remain `0` until a valid GPS position is available.