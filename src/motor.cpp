/**
 * @file motor.cpp
 * @brief High-level motor control abstraction implementation
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements the motor class which provides a unified interface
 *          for controlling multiple motors and encoders in the DEMIR system.
 */

#include "motor.h"

#include "board/DEMIRv1/pins_DEMIRv1.h"

/// @brief Global motor control instance
motor Motor;

/**
 * @brief Default constructor for motor class
 */
motor::motor() {}

/**
 * @brief Initialize all motor drivers and encoders
 * @details Sets up hardware pins and initializes motor drivers and encoders
 *          for all configured motors based on MOTORS macro definition
 */
void motor::init() {
    // Initialize motor 1
    motor_1.init( MOTOR1_INA, MOTOR1_INB, MOTOR1_PWM, MOTOR1_SEL0, MOTOR1_CURRENT_SENSE );
    encoder_1.init( MOTOR1_encPinA, MOTOR1_encPinB );
    
#if MOTORS > 1
    // Initialize motor 2
    motor_2.init( MOTOR2_INA, MOTOR2_INB, MOTOR2_PWM, MOTOR2_SEL0, MOTOR2_CURRENT_SENSE );
    encoder_2.init( MOTOR2_encPinA, MOTOR2_encPinB );
#endif
#if MOTORS > 2
    // Initialize motor 3 (Note: potential bug - uses MOTOR1_PWM instead of MOTOR3_PWM)
    motor_2.init( MOTOR3_INA, MOTOR3_INB, MOTOR1_PWM, MOTOR3_SEL0, MOTOR3_CURRENT_SENSE );
    encoder_3.init( MOTOR3_encPinA, MOTOR3_encPinB );
#endif
#if MOTORS > 3
    // Initialize motor 4
    motor_4.init( MOTOR4_INA, MOTOR4_INB, MOTOR4_PWM, MOTOR4_SEL0, MOTOR4_CURRENT_SENSE );
    encoder_4.init( MOTOR4_encPinA, MOTOR4_encPinB );
#endif
}

/**
 * @brief Set user-defined position direction for encoder
 * @param dir Direction flag (true/false)
 * @param Motor Motor index (0-3)
 * @details Allows reversing the sign of encoder readings without changing hardware
 */
void motor::setUserPosDir( bool dir, uint8_t Motor ) { Encoders[ Motor ]->setUserPosDir( dir ); }

/**
 * @brief Get user-defined position direction setting
 * @param Motor Motor index (0-3)
 * @return Current direction setting
 */
bool motor::getUserPosDir( uint8_t Motor ) { return Encoders[ Motor ]->getUserPosDir(); }

/**
 * @brief Reset encoder position to zero
 * @param Motor Motor index (0-3)
 * @details Zeroes the encoder count, useful for establishing reference positions
 */
void motor::resetPosition( uint8_t Motor ) { Encoders[ Motor ]->resetPosition(); }

/**
 * @brief Set motor speed and direction
 * @param motor_speed Speed value (0-255), sign determines direction
 * @param Motor Motor index (0-3)
 * @details Converts speed command to appropriate PWM and direction signals
 */
void motor::setSpeed( uint8_t motor_speed, uint8_t Motor ) { Motors[ Motor ]->uToPWM( motor_speed ); }

/**
 * @brief Get current motor direction setting
 * @param Motor Motor index (0-3)
 * @return Current direction flag
 */
bool motor::getDir( uint8_t Motor ) { return Motors[ Motor ]->getMotorDir(); }

/**
 * @brief Set motor rotation direction
 * @param dir Direction flag (true/false)
 * @param Motor Motor index (0-3)
 * @details Changes the direction mapping without affecting speed
 */
void motor::setDir( bool dir, uint8_t Motor ) { Motors[ Motor ]->setMotorDir( dir ); }

/**
 * @brief Get current encoder position
 * @param Motor Motor index (0-3)
 * @return Current position in encoder counts
 * @details Returns signed position accounting for user direction setting
 */
int32_t motor::getPosition( uint8_t Motor ) { return Encoders[ Motor ]->getPosition(); }

/**
 * @brief Enable motor driver
 * @param Motor Motor index (0-3)
 * @details Activates the motor driver, allowing motion commands
 */
void motor::enable( uint8_t Motor ) { Motors[ Motor ]->setStatus( ENABLE ); }

/**
 * @brief Disable motor driver
 * @param Motor Motor index (0-3)
 * @details Deactivates the motor driver, preventing motion
 */
void motor::disable( uint8_t Motor ) { Motors[ Motor ]->setStatus( DISABLE ); }

/**
 * @brief Check if motor driver is enabled
 * @param Motor Motor index (0-3)
 * @return True if motor is enabled
 */
bool motor::isEnabled( uint8_t Motor ) { return Motors[ Motor ]->getStatus(); }

/**
 * @brief Check if motor driver is disabled
 * @param Motor Motor index (0-3)
 * @return True if motor is disabled
 */
bool motor::isDisabled( uint8_t Motor ) { return !Motors[ Motor ]->getStatus(); }
