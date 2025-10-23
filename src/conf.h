#ifndef CONF_H
#define CONF_H

// Serial configuration
#define BAUDRATE 115200
#define TIMEOUT 100  // Serial timeout in ms

// Number of motors
#define MOTORS 4

// I2C configuration - multiple controllers
#define MOTOR_CONTROLLER_ADDR_1 0x08
#define MOTOR_CONTROLLER_ADDR_2 0x09
#define MOTOR_CONTROLLER_ADDR_3 0x0A
#define MOTOR_CONTROLLER_ADDR_4 0x0B

// Control loop timing
#define LOOP_RATE_MS 3  // Main control loop rate

#endif // CONF_H
