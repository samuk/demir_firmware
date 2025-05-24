/**
 * @file MotorDriver.cpp
 * @brief Main motor driver orchestration implementation
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements the MotorDriver class which coordinates
 *          motor control, feedback sensing, and control mode execution.
 */

#include "MotorDriver.h"

/// @brief Global motor driver instance
MotorDriver Driver;

/**
 * @brief Initialize the motor driver subsystem
 * @details Initializes motor hardware and control subsystems
 */
void MotorDriver::init() {
    Motor.init();      // Initialize motor hardware
    Controller.init(); // Initialize PID controllers
}

/**
 * @brief Main motor driver execution loop
 * @details Updates sensor readings, executes appropriate control mode,
 *          and applies control outputs to motors. Called periodically
 *          from main control loop.
 */
void MotorDriver::run() {
    // Update sensor feedback
    Controller.updateCurrentPosition();
    Controller.updateCurrentVelocity();

    if ( state() ) {
        // Execute control based on current operating mode
        switch ( getOperatingMode() ) {
            case MANUAL_MODE:  ///< Direct manual control
                Controller.manuelMode();
                break;
            case POSITION_MODE:  ///< Closed-loop position control
                Controller.positionMode();
                break;
            case PROFILE_POSITION_MODE:  ///< Trajectory-based position control
                Controller.profilePositionMode();
                break;
            case VELOCITY_MODE:  ///< Closed-loop velocity control
                Controller.velocityMode();
                break;
            case PROFILE_VELOCITY_MODE:
                // Profile velocity mode - not implemented
                break;
            case CURRENT_MODE:
                // Current control mode - not implemented
                break;
            case HOMING_MODE:
                // Homing mode - not implemented
                break;
        }
    } else {
        // Driver disabled - zero all outputs
        Controller.zeroOutput();
    }
    
    // Apply computed control signals to motors
    Controller.update();
}