# Firefighting Robot Project

## Overview
This repository contains the Arduino code and documentation for the Firefighting Robot project. The goal of this project is to create an autonomous robot that can detect and extinguish fires, offering a novel approach to firefighting in controlled environments.

## Circuit Diagram
Below is the circuit diagram for the Firefighting Robot, showcasing the Arduino integration and other components:
![Taisir Hassan- Firefighting Arduino Project](https://github.com/taisirhassan/Firefighting-Robot/assets/85134103/7b04f41b-bbd0-458d-9987-96bb042ec78c)

## Features
- **Arduino-Controlled**: The robot's core functions are controlled by an Arduino microcontroller.
- **Fire Detection**: Equipped with heat and flame sensors to detect fires.
- **Autonomous Navigation**: Capable of navigating through complex environments autonomously to locate fire sources.
- **Extinguishing System**: Activates an extinguishing mechanism upon detecting fire.

## Components
Key components of this project:
- Arduino Microcontroller
- Heat and Flame Sensors
- Servo Motors for movement
- Fire Extinguishing System Components

## Code Structure
- `main.ino`: The main Arduino sketch that integrates sensor reading, movement, and the motors.

## Setup and Installation

To get your Firefighting Robot operational, follow these steps:

### Hardware Assembly
1. **Assemble the Robot**: Follow the assembly instructions specific to your robot's design. Ensure all mechanical parts like motors, sensors, and the extinguishing system are properly attached.
2. **Wiring**: Connect all the electronic components to the Arduino according to the circuit diagram provided in the repository.
3. **Power Supply**: Ensure a suitable power source is connected to the robot.

### Software Installation
1. **Arduino IDE**: If you haven't already, download and install the Arduino IDE from the [Arduino website](https://www.arduino.cc/en/software).
2. **Clone the Repository**: Download this repository to your computer by cloning it or downloading it as a ZIP file and extracting it.
3. **Open the Sketch**: Navigate to the main Arduino sketch (`main.ino`) in the repository and open it with the Arduino IDE.
4. **Install Libraries**: Install any necessary Arduino libraries that the project depends on. This can typically be done within the Arduino IDE under `Sketch > Include Library > Manage Libraries...`.

### Uploading the Code
1. **Connect the Arduino**: Connect your Arduino to your computer using a USB cable.
2. **Select Arduino Board**: In the Arduino IDE, go to `Tools > Board` and select the model of your Arduino board.
3. **Select Port**: Go to `Tools > Port` and select the COM port that your Arduino is connected to.
4. **Upload the Code**: Click the `Upload` button in the Arduino IDE to compile and upload the sketch to your Arduino.

### Testing
1. **Initial Test**: After the code is uploaded, perform an initial test to ensure the robot is functioning as expected. This might include checking sensor responses, motor movements, and the activation of the extinguishing mechanism.
2. **Safety Precautions**: Ensure all safety precautions are taken, especially when testing the fire detection and extinguishing features.

## Contributing
Contributions to this project are welcome. If you have ideas for improvements or bug fixes, feel free to fork this repository and submit a pull request.

## License
This project is open-source and free for anyone to use and modify.
