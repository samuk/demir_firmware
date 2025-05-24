/**
 * @file MotorDriver.h
 * @brief Main motor driver orchestration class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for MotorDriver class which coordinates motor control,
 *          feedback sensing, and control mode execution for the DEMIR system.
 */

#pragma once

#include "Motion.h"
#include "board/DEMIRv1/hardware.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "conf.h"
#include "controller.h"
#include "motor.h"

/**
 * @enum operatingMode
 * @brief Motor control operating modes
 * @details Defines different control modes available for motor operation
 */
typedef enum : uint8_t {
    MANUAL_MODE = 0,        ///< Direct manual control mode
    POSITION_MODE,          ///< Closed-loop position control
    PROFILE_POSITION_MODE,  ///< Trajectory-based position control
    VELOCITY_MODE,          ///< Closed-loop velocity control
    PROFILE_VELOCITY_MODE,  ///< Trajectory-based velocity control
    CURRENT_MODE,           ///< Current/torque control mode
    HOMING_MODE             ///< Automatic homing mode
} operatingMode;

/**
 * @enum driverState
 * @brief Driver enable/disable states
 * @details Defines the main driver power states for safety control
 */
typedef enum : uint8_t { 
    DRIVER_DISABLE = 0,     ///< Driver disabled (safe state)
    DRIVER_ENABLE           ///< Driver enabled (operational state)
} driverState;

/**
 * @class MotorDriver
 * @brief Main motor driver orchestration class
 * @details Coordinates motor hardware, control algorithms, and operating modes.
 *          Provides high-level interface for motor system management including
 *          initialization, mode selection, and real-time control execution.
 */
class MotorDriver {
   public:
    MotorDriver() = default;  ///< Default constructor
    
    /**
     * @brief Initialize motor driver subsystem
     * @details Sets up motor hardware and control subsystems
     */
    void    init();
    
    /**
     * @brief Get current driver state
     * @return Driver state (DRIVER_ENABLE/DRIVER_DISABLE)
     */
    uint8_t state() { return _state; }
    
    /**
     * @brief Main driver execution function
     * @details Executes control algorithms based on current operating mode
     */
    void    run();
    
    /**
     * @brief Enable the motor driver
     * @details Activates driver and sets to manual mode for safety
     */
    void    enable() {
        _state  = DRIVER_ENABLE;
        _opMode = MANUAL_MODE;
    }
    
    /**
     * @brief Disable the motor driver
     * @details Deactivates driver and sets to manual mode
     */
    void disable() {
        _state  = DRIVER_DISABLE;
        _opMode = MANUAL_MODE;
    }
    
    /**
     * @brief Check if driver is enabled
     * @return True if driver is enabled
     */
    uint8_t isEnabled() { return _state; }
    
    /**
     * @brief Check if driver is disabled
     * @return True if driver is disabled
     */
    uint8_t isDisabled() { return !_state; }

    /**
     * @brief Set motor operating mode
     * @param mode Operating mode (see operatingMode enum)
     */
    void    setOperatingMode( uint8_t mode ) { _opMode = mode; }
    
    /**
     * @brief Get current operating mode
     * @return Current operating mode
     */
    uint8_t getOperatingMode() { return _opMode; }

   private:
    uint8_t _state  = DRIVER_DISABLE;  ///< Current driver state
    uint8_t _opMode = MANUAL_MODE;     ///< Current operating mode
};

/// @brief Global motor driver instance
extern MotorDriver Driver;