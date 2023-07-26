# Motion Detection and Gesture Tracking

This project aims to detect and identifying a multitude of movement types and gestures usic Bluetooth connectivity on a low-power and portable device.

## Table of Contents

- [Features](#features)
- [Bluetooth Profile](#bluetooth-profile)
- [Usage](#usage)
- [License](#license)

## Features

This firmware offers multiple functionalities:

- **Exercise Recognition**: Motion Detection is adept at recognizing physical exercise movements. This provides users with a powerful tool for monitoring and tracking the motion activity.
- **Balance/Variance Feedback**: Motion Detection provides balance and variance score feedback to help users understand their motion control.
- **Artificial Intelligence Algorithms**: Advanced AI algorithms are implemented within the Motion Detection application to accurately identify and categorize a wide range of physical exercises. These algorithms have been trained on a custom dataset, allowing the application to distinguish between various forms of movements and classify them accurately.

## Bluetooth Profile

Motion Detection operates with a custom Bluetooth profile consisting of four unique services:

- **Generic Access Service (UUID 0x1800)**: This service holds general information about the device. It consists of three crucial characteristics: device name (UUID 0x2A00), appearance (UUID 0x2A01), and peripheral preferred connection parameters (UUID 0x2A04).
  
- **Generic Attribute Service (UUID 0x1801)**: The service can be used to notify the central device of changes made to the fundamental structure of services and characteristics on the peripheral.

- **Custom Service (UUID 00000000-ffff-11e1-9ab4-0002a5d5c51b)**: This service provides two characteristics—one for write mode and two for read mode—enabling the activation of process network tasks and alteration of operating modes.

- **Data Service (UUID 00000000-0001-11e1-9ab4-0002a5d5c51b)**: This service allows the reading of data from sensors or from processing. It comprises three characteristics, each delivering unique sensor data.

These services are designed to provide a comprehensive and flexible usage of the device, allowing users to adjust settings, change operating modes, and read data from various sensors.

## Usage

To utilize the Motion Detection software, follow these steps:

1. **Sensor Pairing**: Use the app to pair with your sensor devices via Bluetooth. Multiple sensor devices can be paired if required.
2. **Sensor Setting**: Specify the type of device for each paired sensor.
3. **Feedback Frequency Setting**: Set the frequency for the balance/variance score feedback within the app. For instance, if set to 0.5Hz, the app will provide feedback every two seconds.
4. **Exercise Recognition and Performance Feedback**: This involves recognizing specific movements, repetitions, and unique exercise patterns in real-time. The software then provides immediate feedback on your performance.

## License

This project is licensed under the terms of the MIT license.
