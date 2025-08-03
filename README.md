# Arduino Based Human Following Robot

An intelligent autonomous robot that can detect, track, and follow a human target while maintaining a safe distance using an Arduino microcontroller, sensors, and motor control systems.

## 📋 Table of Contents

1. [Background and Overview](#background-and-overview)
2. [System Architecture](#system-architecture)
3. [Hardware Components](#hardware-components)
4. [Algorithm and Implementation](#algorithm-and-implementation)
5. [Results and Future Scope](#results-and-future-scope)

---

## 1. Background and Overview

### Project Objective
This project demonstrates the development of an intelligent robotic system capable of autonomously tracking and following a human target in dynamic environments. The robot combines hardware sensors, actuators, and intelligent algorithms to create a responsive human-robot interaction system.

### Key Features
- **Autonomous Human Detection**: Uses IR sensors to detect human presence
- **Distance Measurement**: Ultrasonic sensor maintains a safe following distance
- **Intelligent Navigation**: Real-time decision making for movement control
- **Obstacle Avoidance**: Prevents collisions while following target
- **Wireless Potential**: Foundation for future remote monitoring capabilities

### Applications
- **Assistance Robotics**: Personal assistance and mobility aid
- **Surveillance Systems**: Security and monitoring applications
- **Interactive Installations**: Entertainment and educational purposes
- **Research Platform**: Human-robot interaction studies
- **Emergency Response**: Search and rescue operations

### Team Information
- **Institution**: RCC Institute of Information Technology
- **Department**: Electronics and Communication Engineering
- **Supervisor**: Dr. Subhrajit Sinha Roy

**Team Members:**
| Name | Class Roll | University Roll |
|------|------------|-----------------|
| Dwaipayan Bhattacharjee | ECE2021/016 | 11700321071 |
| Trinjan Dutta | ECE2021/033 | 11700321018 |
| Rishav Pramanik | ECE2021/038 | 11700321051 |

---

## 2. System Architecture

### Block Diagram Overview

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   IR Sensors    │───▶│   Arduino UNO   │◄──▶│ Ultrasonic      │
│   (Left/Right)  │    │ (Microcontroller)│    │ Sensor          │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │ Motor Driver    │
                    │ (L298N)         │
                    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │ DC Motors       │───▶│ Wheels &        │
                    │ (300 RPM × 4)   │    │ Chassis         │
                    └─────────────────┘    └─────────────────┘
                              ▲
                              │
                    ┌─────────────────┐
                    │ 12V Battery     │
                    │ Power Supply     │
                    └─────────────────┘
```

### Component Roles
- **Arduino UNO**: Central processing unit managing all system operations
- **Motor Driver (L298N)**: Interface between Arduino and DC motors for speed/direction control
- **DC Motors**: Four 300 RPM Johnson motors providing movement capability
- **Ultrasonic Sensor**: Distance measurement for maintaining safe following distance
- **IR Sensors**: Human heat signature detection for target identification
- **Power Supply**: 12V rechargeable battery system for portable operation

### Circuit Connections

#### Motor Driver L298N Connections:
| L298N Pin | Arduino Pin | Function |
|-----------|-------------|----------|
| ENA | Pin 9 (PWM) | Motor A Speed Control |
| IN1 | Pin 2 | Motor A Direction |
| IN2 | Pin 3 | Motor A Direction |
| IN3 | Pin 4 | Motor B Direction |
| IN4 | Pin 5 | Motor B Direction |
| ENB | Pin 10 (PWM) | Motor B Speed Control |

#### Sensor Connections:
| Sensor | Arduino Pin | Function |
|--------|-------------|----------|
| Ultrasonic Trig | Pin 7 | Distance Trigger |
| Ultrasonic Echo | Pin 6 | Distance Echo |
| Left IR | Pin 11 | Left Side Detection |
| Right IR | Pin 12 | Right Side Detection |

---

## 3. Hardware Components

### Required Components List

| Component | Quantity | Specifications | Purpose |
|-----------|----------|----------------|---------|
| Arduino UNO | 1 | ATmega328P, 16MHz | Main microcontroller |
| Motor Driver L298N | 1 | Dual H-Bridge, 2A | Motor control interface |
| Johnson DC Motors | 4 | 300 RPM, 12V | Locomotion system |
| IR Sensors | 2 | Infrared proximity sensors | Human detection |
| Ultrasonic Sensor | 1 | HC-SR04, 2-400cm range | Distance measurement |
| Jumper Wires | 20+ | Male-to-male/female | Circuit connections |
| Robot Chassis | 1 | Aluminum/Acrylic frame | Structural support |
| Wheels | 4 | Rubber, compatible with motors | Ground contact |
| 12V Battery Pack | 1 | Rechargeable Li-ion/NiMH | Power supply |
| Breadboard | 1 | Half-size recommended | Prototyping connections |

### Component Specifications

#### Arduino UNO
- **Microcontroller**: ATmega328P
- **Operating Voltage**: 5V
- **Input Voltage**: 7-12V (recommended)
- **Digital I/O Pins**: 14 (6 PWM outputs)
- **Analog Input Pins**: 6
- **Flash Memory**: 32KB

#### L298N Motor Driver
- **Logic Voltage**: 5V
- **Motor Voltage**: 5V-35V
- **Logic Current**: 0-36mA
- **Motor Current**: 2A (per channel)
- **Max Power**: 25W

#### Ultrasonic Sensor (HC-SR04)
- **Operating Voltage**: 5V DC
- **Operating Current**: 15mA
- **Working Frequency**: 40Hz
- **Range**: 2cm - 4m
- **Accuracy**: 3mm

---

## 4. Algorithm and Implementation

### System Initialization
1. **Pin Configuration**: Set up motor control pins as outputs and sensor pins as inputs
2. **Serial Communication**: Initialize debugging interface at 9600 baud rate
3. **Motor Calibration**: Ensure all motors are stopped initially
4. **Sensor Validation**: Verify all sensors are responding correctly
5. **System Ready**: 2-second stabilization delay before operation

### Main Control Algorithm

```
START
│
├── READ SENSORS
│   ├── Ultrasonic Distance Measurement
│   └── IR Sensor Detection (Left/Right)
│
├── DECISION MAKING LOGIC
│   │
│   ├── IF Distance < MIN_DISTANCE (10cm)
│   │   └── EMERGENCY STOP → MOVE BACKWARD
│   │
│   ├── IF Right IR Detected AND Left IR Clear
│   │   └── TURN RIGHT
│   │
│   ├── IF Left IR Detected AND Right IR Clear
│   │   └── TURN LEFT
│   │
│   ├── IF Both IR Sensors Detect Human
│   │   ├── IF Distance in SAFE_RANGE (20-50cm)
│   │   │   └── MOVE FORWARD
│   │   ├── IF Distance > MAX_DISTANCE
│   │   │   └── MOVE FORWARD (APPROACH)
│   │   └── ELSE → MAINTAIN POSITION
│   │
│   └── IF No IR Detection
│       ├── IF Object in Ultrasonic Range
│       │   └── MOVE FORWARD SLOWLY
│       └── ELSE → STOP AND WAIT
│
├── MOTOR CONTROL EXECUTION
│   ├── Set Motor Directions
│   ├── Apply PWM Speed Control
│   └── Monitor Movement
│
└── LOOP BACK TO START
```

### Movement Functions

#### Forward Movement
- Both motors rotate in forward direction
- Equal speed for straight line motion
- Speed adjustable based on distance

#### Turning Operations
- **Turn Left**: Reduce left motor speed, maintain right motor speed
- **Turn Right**: Reduce right motor speed, maintain left motor speed
- **Sharp Turns**: Reverse one motor while maintaining forward on other

#### Safety Features
- **Collision Avoidance**: Immediate stop when obstacle too close
- **Speed Regulation**: Variable speed based on target distance
- **Emergency Backup**: Reverse movement when critically close

### Sensor Processing

#### Ultrasonic Distance Calculation
```cpp
distance = (pulse_duration * 0.034) / 2;
```
- **Pulse Duration**: Time taken for sound wave to return
- **Speed of Sound**: 343 m/s (0.034 cm/µs)
- **Distance Formula**: (Time × Speed) ÷ 2 (round trip)

#### IR Sensor Logic
- **HIGH Signal**: Human heat signature detected
- **LOW Signal**: No detection or obstacle
- **Dual Sensor**: Determines direction of human movement

### Code Structure

#### Key Constants
```cpp
#define SAFE_DISTANCE 20    // Optimal following distance (cm)
#define MAX_DISTANCE 50     // Maximum detection range (cm)
#define MIN_DISTANCE 10     // Collision avoidance threshold (cm)
#define MOTOR_SPEED 150     // Base movement speed (0-255)
#define TURN_SPEED 100      // Turning operation speed
```

#### Main Functions
- `readSensors()`: Collect data from all sensors
- `makeDecision()`: Process sensor data and determine action
- `moveForward()`: Execute forward movement
- `turnLeft()/turnRight()`: Execute turning movements
- `stopMotors()`: Emergency stop function

---

## 5. Results and Future Scope

### Experimental Results

#### Testing Phases
1. **Component Testing**: Individual sensor and motor validation
2. **Integration Testing**: Combined system functionality verification
3. **Distance Calibration**: Optimal following distance determination
4. **Movement Testing**: Navigation accuracy and responsiveness evaluation
5. **Real-world Testing**: Human following performance in various environments

#### Performance Metrics
- **Detection Range**: 2-50 cm effective range
- **Following Accuracy**: ±5 cm distance maintenance
- **Response Time**: <200ms for direction changes
- **Battery Life**: 2-3 hours continuous operation
- **Success Rate**: 95% human tracking accuracy in a controlled environment

#### Test Results Summary
| Test Parameter | Result | Status |
|----------------|---------|---------|
| Distance Maintenance | 20±3 cm | ✅ Excellent |
| Direction Tracking | 95% accuracy | ✅ Excellent |
| Obstacle Avoidance | 100% collision-free | ✅ Perfect |
| Battery Performance | 2.5 hours average | ✅ Good |
| Response Time | 150ms average | ✅ Excellent |

### Future Enhancements

#### Short-term Improvements (1-3 months)
1. **Enhanced Sensor Array**
   - Add additional IR sensors for 360° detection
   - Implement camera module for visual tracking
   - Include a gyroscope for better orientation control

2. **Algorithm Optimization**
   - Implement PID control for smoother movement
   - Add predictive tracking for faster humans
   - Develop machine learning for behavior adaptation

#### Medium-term Developments (3-6 months)
1. **Wireless Communication**
   - **Bluetooth Integration**: Remote monitoring and control
   - **Wi-Fi Connectivity**: Internet-based operation and data logging
   - **Mobile App**: Real-time status and control interface

2. **Advanced Navigation**
   - **SLAM Implementation**: Simultaneous Localization and Mapping
   - **Path Planning**: Optimized route calculation
   - **Multi-target Tracking**: Follow multiple humans simultaneously

#### Long-term Vision (6-12 months)
1. **Military and Security Applications**
   - **Surveillance Capabilities**: Real-time video recording and transmission
   - **Perimeter Monitoring**: Autonomous patrol functionality
   - **Threat Detection**: Integration with security systems

2. **Disaster Management**
   - **Search and Rescue**: Human detection in disaster scenarios
   - **Medical Emergency**: First aid supply delivery
   - **Environmental Monitoring**: Data collection in hazardous areas

3. **Multi-Robot Coordination**
   - **Swarm Intelligence**: Coordinated multi-robot operations
   - **Task Distribution**: Collaborative problem-solving
   - **Communication Network**: Inter-robot data sharing

### Practical Applications

#### Healthcare and Assistance
- **Elderly Care**: Personal assistance and mobility support
- **Hospital Navigation**: Guide patients and visitors
- **Rehabilitation**: Physical therapy assistance

#### Commercial and Industrial
- **Warehouse Operations**: Inventory tracking and management
- **Manufacturing**: Worker safety and assistance
- **Retail**: Customer service and navigation

#### Educational and Research
- **STEM Education**: Robotics learning platform
- **Research Tool**: Human-robot interaction studies
- **Competition Platform**: Robotics contests and demonstrations

### Technical Challenges and Solutions

#### Current Limitations
1. **Environmental Sensitivity**: IR sensors affected by ambient light
2. **Battery Life**: Limited operational duration
3. **Terrain Limitations**: Smooth surface requirement
4. **Single Target**: Can only follow one person at a time

#### Proposed Solutions
1. **Sensor Fusion**: Combine multiple sensor types for reliability
2. **Power Management**: Implement sleep modes and energy optimization
3. **Adaptive Control**: Terrain-specific movement algorithms
4. **Computer Vision**: Camera-based multi-target tracking

---

## 🚀 Getting Started

### Prerequisites
- Arduino IDE (version 1.8.0 or higher)
- Basic understanding of electronics and programming
- Soldering equipment (for permanent connections)
- Multimeter for troubleshooting

### Installation Steps
1. **Hardware Assembly**
   ```
   1. Mount motors on chassis
   2. Install wheels on motors
   3. Connect the motor driver to Arduino
   4. Mount sensors on the front of the chassis
   5. Connect power supply
   6. Secure all connections
   ```

2. **Software Setup**
   ```
   1. Download Arduino IDE
   2. Connect Arduino UNO via USB
   3. Select the correct board and port
   4. Upload the provided code
   5. Open Serial Monitor for debugging
   ```

3. **Calibration Process**
   ```
   1. Test individual motor movements
   2. Calibrate sensor readings
   3. Adjust distance thresholds
   4. Fine-tune movement speeds
   5. Validate complete system operation
   ```

### Usage Instructions
1. **Power On**: Connect 12V battery and Arduino power
2. **Initialization**: Wait for system startup (2 seconds)
3. **Positioning**: Place robot behind target human
4. **Operation**: The Robot will automatically detect and follow
5. **Monitoring**: Use Serial Monitor for real-time status

### Troubleshooting Guide

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Robot doesn't move | Power/connection issue | Check battery and motor connections |
| Erratic movement | Sensor interference | Recalibrate sensors, check wiring |
| Poor following | Distance threshold | Adjust SAFE_DISTANCE constant |
| No detection | IR sensor failure | Test sensors individually |
| Continuous rotation | Wiring error | Verify motor driver connections |

---

## 📊 Technical Specifications

### Performance Parameters
- **Maximum Speed**: 1.2 m/s
- **Turning Radius**: 30 cm minimum
- **Detection Range**: 2-50 cm
- **Operating Voltage**: 12V DC
- **Current Consumption**: 2-3A (motors active)
- **Weight**: 2.5 kg approximately
- **Dimensions**: 25cm × 20cm × 15cm

### Environmental Conditions
- **Operating Temperature**: 0°C to 50°C
- **Humidity**: 20% to 80% (non-condensing)
- **Surface**: Flat to moderate inclines (up to 15°)
- **Lighting**: Indoor/outdoor (IR sensors may vary)

---

## 🤝 Contributing

We welcome contributions to improve this project! Areas for development:
- Enhanced algorithms for better tracking
- Additional sensor integration
- Power optimization techniques
- User interface improvements
- Documentation enhancements

### How to Contribute
1. Fork the repository
2. Create a feature branch
3. Make your improvements
4. Test thoroughly
5. Submit a pull request with a detailed description

---

## 📝 License

This project is developed for educational purposes at RCC Institute of Information Technology. Feel free to use and modify for academic and research purposes.

---

## 👥 Authors and Acknowledgments

**Development Team:**
- **Dwaipayan Bhattacharjee** - Hardware Integration and Testing
- **Trinjan Dutta** - Software Development and Algorithm Design  
- **Rishav Pramanik** - Circuit Design and Documentation

**Special Thanks:**
- **Dr. Subhrajit Sinha Roy** - Project Supervision and Guidance
- **RCC Institute of Information Technology** - Laboratory and Resources
- **Department of ECE** - Technical Support and Facilities

---


---

*This project demonstrates the practical application of embedded systems, sensor technology, and robotics in creating intelligent autonomous systems for human-robot interaction.*
