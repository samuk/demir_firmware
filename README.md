# DEMIR (Dynamically Enhanced Motor Intelligence for Robots)

A sophisticated closed-loop motor driver platform designed for precise robotic control applications.

## 📋 Overview

DEMIR is an advanced motor control system developed as a final year engineering project. It combines hardware and software to deliver high-precision motor control with intelligent feedback mechanisms.

### Key Features

- **Closed-loop Control**: Encoder-based PID position controller
- **Profile Generation**: Smooth motion profiles for precise movements
- **Multi-motor Support**: Control up to 4 motors simultaneously
- **Real-time Communication**: UART/Serial interface for commands
- **Hardware Integration**: Built around ATmega328P microcontroller and VNH7070 motor driver

## 🔧 Hardware Specifications

- **Microcontroller**: ATmega328P (Arduino UNO compatible)
- **Motor Driver**: VNH7070 H-bridge driver
- **Encoder Support**: Quadrature encoder input
- **Communication**: UART (9600 baud default)
- **Power Supply**: 12V DC input for motors
- **Operating Voltage**: 5V logic level

## 🚀 Software Features

### Motion Control
- **PID Controller**: Tunable proportional, integral, and derivative gains
- **Velocity Profiling**: Configurable acceleration and deceleration ramps
- **Position Accuracy**: High-precision positioning with encoder feedback
- **Real-time Monitoring**: Live position and velocity feedback

### Communication Protocol
- Serial command interface
- Status reporting
- Parameter configuration
- Emergency stop functionality

## 📁 Project Structure

```
.
├── .github
│   └── workflows
│       └── build.yml
├── include
│   ├── board
│   │   └── DEMIRv1
│   │       ├── hardware.h
│   │       └── pins_DEMIRv1.h
│   ├── com_def.h
│   ├── conf.h
│   ├── controller.h
│   ├── encoder.h
│   ├── Motion.h
│   ├── motor.h
│   ├── MotorDriver.h
│   ├── PID.h
│   ├── PLANNER.h
│   ├── README
│   ├── SerialParser.h
│   ├── SerialSolver.h
│   └── vnh7070.h
├── src
│   ├── com_def.cpp
│   ├── controller.cpp
│   ├── encoder.cpp
│   ├── main.cpp
│   ├── Motion.cpp
│   ├── motor.cpp
│   ├── MotorDriver.cpp
│   ├── PID.cpp
│   ├── PLANNER.cpp
│   ├── README
│   ├── SerialParser.cpp
│   ├── SerialSolver.cpp
│   └── vnh7070.cpp
├── .clang-format
├── .gitignore
├── .travis.yml
├── LICENSE
├── platformio.ini
└── README.md
```

## 🛠️ Building and Installation

### Prerequisites
- Arduino IDE or Arduino CLI
- ATmega328P compatible board (Arduino UNO)
- C++14 compatible compiler

### Build Instructions

1. **Clone the repository**:
   ```bash
   git clone https://github.com/dsm/demir_firmware.git
   cd demir_firmware
   ```

2. **Using Arduino IDE**:
   - Open `src/main.cpp` in Arduino IDE
   - Select Arduino UNO board
   - Upload to your device

3. **Using Arduino CLI**:
   ```bash
   arduino-cli compile --fqbn arduino:avr:uno --build-property "compiler.cpp.extra_flags=-std=c++14"
   arduino-cli upload --fqbn arduino:avr:uno --port /dev/ttyUSB0
   ```

## ⚙️ Configuration

### Motor Settings
Configure motor parameters in `include/conf.h`:

```cpp
#define MOTORS 4              // Number of motors (1-4)
#define ENCODER_PPR 1024      // Pulses per revolution
#define MAX_VELOCITY 1000     // Maximum velocity (pulses/sec)
#define PID_KP 2.0           // Proportional gain
#define PID_KI 0.1           // Integral gain
#define PID_KD 0.05          // Derivative gain
```

### Communication Settings
- **Baud Rate**: 9600 (configurable)
- **Data Format**: 8N1
- **Flow Control**: None

## 📡 Command Protocol

### Basic Commands
- `M<id> P<position>` - Move motor to position
- `M<id> V<velocity>` - Set motor velocity
- `M<id> S` - Stop motor
- `STATUS` - Get system status
- `RESET` - Emergency reset

### Example Usage
```
M1 P1000    # Move motor 1 to position 1000
M2 V500     # Set motor 2 velocity to 500
STATUS      # Get current status
```

## 🔍 Monitoring and Debugging

### Status Information
The system provides real-time feedback including:
- Current position for each motor
- Target position
- Current velocity
- PID error values
- System status

### Debug Output
Enable debug mode in configuration for detailed logging:
```cpp
#define DEBUG_MODE 1
```

## 🧪 Testing

### Unit Tests
Run automated tests using the GitHub Actions workflow:
- Compilation verification
- Code linting
- Memory usage analysis

### Manual Testing
1. Connect motors and encoders
2. Upload firmware
3. Use serial monitor for command testing
4. Verify position accuracy and response

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📊 Performance Metrics

- **Position Accuracy**: ±1 encoder pulse
- **Response Time**: <10ms for position commands
- **Maximum Update Rate**: 1kHz control loop
- **Memory Usage**: ~26% of ATmega328P flash memory

## 🔧 Troubleshooting

### Common Issues
1. **Motor not moving**: Check power supply and wiring
2. **Position drift**: Verify encoder connections
3. **Communication errors**: Check baud rate and serial settings
4. **Compilation errors**: Ensure C++14 support is enabled

### Debug Steps
1. Enable debug output
2. Monitor serial communication
3. Check hardware connections
4. Verify configuration parameters

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Author

**[Muhammet Şükrü Demir]** - Final Year Engineering Project

## 🙏 Acknowledgments

- Arduino community for excellent development tools
- VNH7070 documentation and support
- University advisors and project mentors

---

**Note**: This project is part of a final year engineering project and is actively under development. Features and documentation are continuously being improved.
