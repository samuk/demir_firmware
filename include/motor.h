/**
 * @file motor.h
 * @brief High-level motor control abstraction class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for motor class which provides unified interface for
 *          controlling multiple motors and encoders in the DEMIR system.
 */

#pragma once

#include "PID.h"
#include "board/DEMIRv1/hardware.h"
#include "conf.h"
#include "encoder.h"
#include "vnh7070.h"

/**
 * @enum cfgMotors
 * @brief Motor index enumeration
 * @details Defines motor indices for multi-motor systems
 */
typedef enum : uint8_t {
    MOTOR_1 = 0,  ///< Motor 1 index
    MOTOR_2,      ///< Motor 2 index
    MOTOR_3,      ///< Motor 3 index
    MOTOR_4       ///< Motor 4 index
} cfgMotors;

/**
 * @class motor
 * @brief High-level motor control abstraction
 * @details Provides unified interface for motor control operations including
 *          speed control, direction control, position feedback, and enable/disable
 *          functionality. Abstracts hardware details and provides consistent API.
 */
class motor {
   public:
    /**
     * @brief Default constructor
     */
    motor();

    /**
     * @brief Initialize all motor hardware
     * @details Sets up motor drivers and encoders for all configured motors
     */
    void init();

    /**
     * @brief Set motor speed and direction
     * @param motor_speed Speed value (0-255), sign determines direction
     * @param Motor Motor index (0-3)
     */
    void setSpeed( const uint8_t motor_speed, uint8_t Motor );

    /**
     * @brief Set motor rotation direction
     * @param dir Direction flag (true/false)
     * @param Motor Motor index (0-3)
     */
    void setDir( bool dir, uint8_t Motor );

    /**
     * @brief Get current motor direction setting
     * @param Motor Motor index (0-3)
     * @return Current direction flag
     */
    bool getDir( uint8_t Motor );

    /**
     * @brief Enable motor driver
     * @param Motor Motor index (0-3)
     */
    void enable( uint8_t Motor );

    /**
     * @brief Disable motor driver
     * @param Motor Motor index (0-3)
     */
    void disable( uint8_t Motor );

    /**
     * @brief Check if motor is enabled
     * @param Motor Motor index (0-3)
     * @return True if motor is enabled
     */
    bool isEnabled( uint8_t Motor );

    /**
     * @brief Check if motor is disabled
     * @param Motor Motor index (0-3)
     * @return True if motor is disabled
     */
    bool isDisabled( uint8_t Motor );

    /**
     * @brief Get current encoder position
     * @param Motor Motor index (0-3)
     * @return Position in encoder counts
     */
    int32_t getPosition( uint8_t Motor );

    /**
     * @brief Reset encoder position to zero
     * @param Motor Motor index (0-3)
     */
    void    resetPosition( uint8_t Motor );

    /**
     * @brief Set user-defined position direction
     * @param dir Direction flag (true/false)
     * @param Motor Motor index (0-3)
     */
    void    setUserPosDir( bool dir, uint8_t Motor );

    /**
     * @brief Get user-defined position direction
     * @param Motor Motor index (0-3)
     * @return Current direction setting
     */
    bool    getUserPosDir( uint8_t Motor );

   private:
    // No private members currently used
};

/// @brief Global motor control instance
extern motor Motor;
