/**
 * @file Motion.h
 * @brief Motion planning coordination class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for Motion class which provides unified interface to
 *          trajectory planning functionality for coordinated multi-axis motion.
 */

#pragma once

#include "PID.h"
#include "PLANNER.h"
#include "board/DEMIRv1/hardware.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "conf.h"
#include "controller.h"
#include "motor.h"

/**
 * @class Motion
 * @brief Motion planning coordination class
 * @details Provides high-level interface to trajectory planning functionality.
 *          Coordinates multiple PLANNER instances for synchronized multi-axis motion
 *          with polynomial trajectory generation and real-time execution.
 */
class Motion {
   public:
    Motion() = default;  ///< Default constructor
    
    /**
     * @brief Initialize motion planning subsystem
     * @details Sets up trajectory planners for all configured axes
     */
    void init();

    /**
     * @brief Enable trajectory planning for specified motor
     * @param Motor Motor index (0-3)
     */
    void enable( uint8_t Motor );
    
    /**
     * @brief Disable trajectory planning for all motors
     */
    void disable();
    
    /**
     * @brief Disable trajectory planning for specified motor
     * @param Motor Motor index (0-3)
     */
    void disable( uint8_t Motor );
    
    /**
     * @brief Check if trajectory planning is enabled
     * @param Motor Motor index (0-3)
     * @return True if trajectory planning is active
     */
    bool isEnabled( uint8_t Motor );
    
    /**
     * @brief Check if trajectory planning is disabled
     * @param Motor Motor index (0-3)
     * @return True if trajectory planning is inactive
     */
    bool isDisabled( uint8_t Motor );

    // Trajectory parameter setters
    void setRelativePositionReference( const int32_t value, uint8_t Motor );  ///< Set relative position change
    void setInitialTime( const uint32_t value, uint8_t Motor );              ///< Set trajectory start time
    void setInitialPosition( const int32_t value, uint8_t Motor );           ///< Set starting position
    void setFinalPosition( const int32_t value, uint8_t Motor );             ///< Set target position
    void setTimeInSec( const float value, uint8_t Motor );                   ///< Set trajectory duration
    void setRelativeTime( const float value, uint8_t Motor );                ///< Set current execution time
    void setPositionModeFired( const bool value, uint8_t Motor );            ///< Set trajectory active flag

    // Trajectory parameter getters
    int32_t  getRelativePositionReference( uint8_t Motor );  ///< Get relative position change
    uint32_t getInitialTime( uint8_t Motor );               ///< Get trajectory start time
    int32_t  getInitialPosition( uint8_t Motor );           ///< Get starting position
    int32_t  getFinalPosition( uint8_t Motor );             ///< Get target position
    float    getTimeInSec( uint8_t Motor );                 ///< Get trajectory duration
    float    getRelativeTime( uint8_t Motor );              ///< Get current execution time
    bool     getPositionModeFired( uint8_t Motor );         ///< Get trajectory active flag

    /**
     * @brief Generate polynomial trajectory profile
     * @param Motor Motor index (0-3)
     */
    void  generatePosProfile( uint8_t Motor );
    
    /**
     * @brief Evaluate position at specified time
     * @param evalTime Time to evaluate (seconds)
     * @param Motor Motor index (0-3)
     * @return Position at specified time
     */
    float evalPos( float evalTime, uint8_t Motor );
    
    /**
     * @brief Evaluate velocity at specified time
     * @param evalTime Time to evaluate (seconds)
     * @param Motor Motor index (0-3)
     * @return Velocity at specified time
     */
    float evalVel( float evalTime, uint8_t Motor );
    
    /**
     * @brief Evaluate acceleration at specified time
     * @param evalTime Time to evaluate (seconds)
     * @param Motor Motor index (0-3)
     * @return Acceleration at specified time
     */
    float evalAccel( float evalTime, uint8_t Motor );
};

/// @brief Global motion planner instance
extern Motion Planner;