# ProjectBusWise

ProjectBusWise is an embedded system designed for real-time monitoring of a bus's motion parameters to detect accidents and over-speed conditions. Using an Arduino-compatible microcontroller, MPU6050 sensor (gyroscope and accelerometer), and GSM/GPS modules, it calculates roll, pitch, and speed to identify dangerous events and sends alerts via SMS.

## Features

- **Accident Detection**: Continuously monitors roll and pitch angles using gyroscope and accelerometer data with Kalman filtering. If thresholds are exceeded, it triggers an accident alert.
- **Over-speed Monitoring**: Calculates the bus's velocity using inertial measurements. If the velocity exceeds a set limit, a speed alert is sent.
- **SMS Alerts**: Sends SMS notifications through a GSM module to a predefined phone number in the event of an accident or when the speed limit is exceeded.
- **Real-time Data Output**: Prints sensor readings and calculated parameters (angles, velocities) to the serial monitor for debugging and monitoring.
- **GPS Integration**: Interfaces with a GPS module for location data, which can be extended to include location information in alerts.

## Hardware Requirements

- Arduino-compatible microcontroller
- MPU6050 Gyroscope & Accelerometer sensor
- SIM900A GSM module (or compatible)
- GPS module (connected via SoftwareSerial)
- Required jumper wires and breadboard

## How It Works

1. **Sensor Initialization**: Sets up communication with the MPU6050 sensor and initializes serial connections for GSM and GPS modules.
2. **Sensor Reading**: Reads acceleration and gyroscope data, calculates roll and pitch angles using a Kalman filter for accuracy.
3. **Accident & Speed Detection**: Compares calculated angles and velocity against predefined thresholds:
    - If an accident (excessive tilt) is detected, sends an SMS alert.
    - If the speed limit is exceeded, sends a speed alert.
4. **Data Communication**: Continuously listens for GPS data and can relay commands between the serial port and GPS module.

## Code Overview

The main logic is contained in `BusWise.ino`, which includes:
- Reading and filtering sensor data.
- Calculating inertial velocities and orientation.
- Detecting threshold violations.
- Communicating with GSM and GPS modules.

## Usage

1. Connect all hardware components as per their respective pin assignments in the code.
2. Upload `BusWise.ino` to your Arduino board.
3. Open the serial monitor at 57600 baud to observe real-time data.
4. On accident or over-speed events, the system will send SMS alerts to the configured phone number.

## Configuration

- **Thresholds**: Roll and pitch angle thresholds, as well as the velocity limit, can be adjusted in the code:
    ```cpp
    const float RollThreshould = 15.0;
    const float PitchThreshould = 10.0;
    const float VelocityLim = 80.0;
    ```
- **Alert Phone Number**: Update the phone number in this line to your desired recipient:
    ```cpp
    SIM900A.print("AT+CMGS=\"+94725263276\"\r");
    ```

## Extending

- Include GPS coordinates in SMS alerts by parsing GPS data in the main loop.
- Integrate with an IoT platform for cloud-based monitoring.
- Add support for other events or integrate more sensors for advanced analytics.
