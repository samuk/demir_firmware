/**
 * @file conf.h
 * @brief Global configuration definitions for DEMIR motor controller
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file contains all system-wide configuration parameters,
 *          hardware pin definitions, and compile-time constants.
 */

#pragma once

/**
 * @def FORCE_INLINE
 * @brief Force function inlining for performance-critical code
 */
#define FORCE_INLINE __attribute__( ( always_inline ) ) inline

/**
 * @def TIMER1_A_PIN
 * @brief Timer1 channel A output pin (Arduino pin 9)
 */
#define TIMER1_A_PIN 9

/**
 * @def TIMER1_B_PIN
 * @brief Timer1 channel B output pin (Arduino pin 10)
 */
#define TIMER1_B_PIN 10

/**
 * @def VNH7070
 * @brief Motor driver IC type identifier
 */
#define VNH7070

/**
 * @def BAUDRATE
 * @brief Serial communication baud rate (500000 bps)
 */
#define BAUDRATE 500000

/**
 * @def TIMEOUT
 * @brief Serial communication timeout in milliseconds
 */
#define TIMEOUT  100

/**
 * @def ANALOG_WRITE_RES
 * @brief PWM resolution in bits (9-bit = 512 levels)
 */
#define ANALOG_WRITE_RES 9

/**
 * @def MOTORS
 * @brief Number of motors in the system
 */
#define MOTORS 1

/**
 * @def PWMS
 * @brief Number of PWM channels available
 */
#define PWMS   6

/**
 * @def MAX_PROF_ACCEL
 * @brief Maximum profile acceleration in encoder pulses per second squared
 */
#define MAX_PROF_ACCEL 19000.0

/**
 * @def MAX_PROF_VEL
 * @brief Maximum profile velocity in encoder pulses per second
 */
#define MAX_PROF_VEL   19000.0

/**
 * @def LOG_SIZE_MAX
 * @brief Maximum size of data logging buffer
 */
#define LOG_SIZE_MAX 200

/**
 * @def LOG_POSITION
 * @brief Logging mode identifier for position data
 */
#define LOG_POSITION 0

/**
 * @def LOG_CURRENT
 * @brief Logging mode identifier for current data
 */
#define LOG_CURRENT  1

/**
 * @def DISABLED
 * @brief Driver disabled state identifier
 */
#define DISABLED 0

/**
 * @def ENABLED
 * @brief Driver enabled state identifier
 */
#define ENABLED  1

/**
 * @def DISABLE
 * @brief Generic disable command
 */
#define DISABLE 0

/**
 * @def ENABLE
 * @brief Generic enable command
 */
#define ENABLE  1

/**
 * @def STATUS_LED_BLUE
 * @brief Blue status LED pin assignment
 */
#define STATUS_LED_BLUE 5

/**
 * @def STATUS_LED_RED
 * @brief Red status LED pin assignment
 */
#define STATUS_LED_RED  6

// Motor 1 pin definitions
#define MOTOR1_CURRENT_SENSE A1  ///< @brief Motor 1 current sensing pin
#define MOTOR1_INA           8   ///< @brief Motor 1 direction input A
#define MOTOR1_INB           7   ///< @brief Motor 1 direction input B
#define MOTOR1_PWM           9   ///< @brief Motor 1 PWM speed control
#define MOTOR1_SEL0          4   ///< @brief Motor 1 current sense range select
#define MOTOR1_encPinA       2   ///< @brief Motor 1 encoder channel A
#define MOTOR1_encPinB       3   ///< @brief Motor 1 encoder channel B

// Motor 2 pin definitions (undefined - to be configured)
#define MOTOR2_CURRENT_SENSE  ///< @brief Motor 2 current sensing pin (undefined)
#define MOTOR2_INA            ///< @brief Motor 2 direction input A (undefined)
#define MOTOR2_INB            ///< @brief Motor 2 direction input B (undefined)
#define MOTOR2_PWM            ///< @brief Motor 2 PWM speed control (undefined)
#define MOTOR2_SEL0           ///< @brief Motor 2 current sense range select (undefined)
#define MOTOR2_encPinA        ///< @brief Motor 2 encoder channel A (undefined)
#define MOTOR2_encPinB        ///< @brief Motor 2 encoder channel B (undefined)

// Motor 3 pin definitions (undefined - to be configured)
#define MOTOR3_CURRENT_SENSE  ///< @brief Motor 3 current sensing pin (undefined)
#define MOTOR3_INA            ///< @brief Motor 3 direction input A (undefined)
#define MOTOR3_INB            ///< @brief Motor 3 direction input B (undefined)
#define MOTOR3_PWM            ///< @brief Motor 3 PWM speed control (undefined)
#define MOTOR3_SEL0           ///< @brief Motor 3 current sense range select (undefined)
#define MOTOR3_encPinA        ///< @brief Motor 3 encoder channel A (undefined)

// Motor 4 pin definitions (undefined - to be configured)
#define MOTOR4_CURRENT_SENSE  ///< @brief Motor 4 current sensing pin (undefined)
#define MOTOR4_INA            ///< @brief Motor 4 direction input A (undefined)
#define MOTOR4_INB            ///< @brief Motor 4 direction input B (undefined)
#define MOTOR4_PWM            ///< @brief Motor 4 PWM speed control (undefined)
#define MOTOR4_SEL0           ///< @brief Motor 4 current sense range select (undefined)
#define MOTOR4_encPinA        ///< @brief Motor 4 encoder channel A (undefined)
#define MOTOR4_encPinB        ///< @brief Motor 4 encoder channel B (undefined)