/**
 * @file Motion.cpp
 * @brief Motion planning coordination implementation
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements the Motion class which provides a unified interface
 *          to trajectory planning functionality for coordinated multi-axis motion.
 */

#include "Motion.h"

/// @brief Global motion planner instance
Motion Planner;

/**
 * @brief Enable trajectory planning for specified motor
 * @param Motor Motor index (0-3)
 * @details Activates trajectory execution for the specified motor axis
 */
void Motion::enable( uint8_t Motor ) { Planners[ Motor ]->setPositionModeFired( true ); }

/**
 * @brief Disable trajectory planning for all motors
 * @details Deactivates trajectory execution for all motor axes
 */
void Motion::disable() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        Planners[ _Motor ]->setPositionModeFired( false );
    }
}

/**
 * @brief Disable trajectory planning for specified motor
 * @param Motor Motor index (0-3)
 * @details Deactivates trajectory execution for the specified motor axis
 */
void Motion::disable( uint8_t Motor ) { Planners[ Motor ]->setPositionModeFired( true ); }

/**
 * @brief Check if trajectory planning is enabled for motor
 * @param Motor Motor index (0-3)
 * @return True if trajectory planning is active
 */
bool Motion::isEnabled( uint8_t Motor ) { return Planners[ Motor ]->getPositionModeFired(); }

/**
 * @brief Check if trajectory planning is disabled for motor
 * @param Motor Motor index (0-3)
 * @return True if trajectory planning is inactive
 */
bool Motion::isDisabled( uint8_t Motor ) { return !Planners[ Motor ]->getPositionModeFired(); }

/**
 * @brief Set relative position reference for trajectory
 * @param value Relative position change in encoder counts
 * @param Motor Motor index (0-3)
 * @details Sets the position change for trajectory generation
 */
void Motion::setRelativePositionReference( const int32_t value, uint8_t Motor ) {
    Planners[ Motor ]->setRelativePositionReference( value );
}

/**
 * @brief Set trajectory start time
 * @param value Start time in milliseconds
 * @param Motor Motor index (0-3)
 * @details Records when trajectory execution should begin
 */
void Motion::setInitialTime( const uint32_t value, uint8_t Motor ) { Planners[ Motor ]->setInitialTime( value ); }

/**
 * @brief Set trajectory starting position
 * @param value Initial position in encoder counts
 * @param Motor Motor index (0-3)
 * @details Sets the starting point for trajectory calculation
 */
void Motion::setInitialPosition( const int32_t value, uint8_t Motor ) {
    Planners[ Motor ]->setInitialPosition( value );
}

/**
 * @brief Set trajectory ending position
 * @param value Final position in encoder counts
 * @param Motor Motor index (0-3)
 * @details Sets the target endpoint for trajectory
 */
void Motion::setFinalPosition( const int32_t value, uint8_t Motor ) { Planners[ Motor ]->setFinalPosition( value ); }

/**
 * @brief Set trajectory duration
 * @param value Duration in seconds
 * @param Motor Motor index (0-3)
 * @details Sets the time required to complete the trajectory
 */
void Motion::setTimeInSec( const float value, uint8_t Motor ) { Planners[ Motor ]->setTimeInSec( value ); }

/**
 * @brief Set current relative time within trajectory
 * @param value Current time offset in milliseconds
 * @param Motor Motor index (0-3)
 * @details Updates the current execution time for trajectory evaluation
 */
void Motion::setRelativeTime( const float value, uint8_t Motor ) { Planners[ Motor ]->setRelativeTime( value ); }

/**
 * @brief Get relative position reference
 * @param Motor Motor index (0-3)
 * @return Relative position change in encoder counts
 */
int32_t Motion::getRelativePositionReference( uint8_t Motor ) {
    return Planners[ Motor ]->getRelativePositionReference();
}

/**
 * @brief Get trajectory start time
 * @param Motor Motor index (0-3)
 * @return Start time in milliseconds
 */
uint32_t Motion::getInitialTime( uint8_t Motor ) { return Planners[ Motor ]->getInitialTime(); }

/**
 * @brief Get trajectory starting position
 * @param Motor Motor index (0-3)
 * @return Initial position in encoder counts
 */
int32_t Motion::getInitialPosition( uint8_t Motor ) { return Planners[ Motor ]->getInitialPosition(); }

/**
 * @brief Get trajectory ending position
 * @param Motor Motor index (0-3)
 * @return Final position in encoder counts
 */
int32_t Motion::getFinalPosition( uint8_t Motor ) { return Planners[ Motor ]->getFinalPosition(); }

/**
 * @brief Get trajectory duration
 * @param Motor Motor index (0-3)
 * @return Duration in seconds
 */
float Motion::getTimeInSec( uint8_t Motor ) { return Planners[ Motor ]->getTimeInSec(); }

/**
 * @brief Get current relative time within trajectory
 * @param Motor Motor index (0-3)
 * @return Current time offset in milliseconds
 */
float Motion::getRelativeTime( uint8_t Motor ) { return Planners[ Motor ]->getRelativeTime(); }

/**
 * @brief Generate polynomial trajectory profile
 * @param Motor Motor index (0-3)
 * @details Calculates 5th order polynomial coefficients for smooth motion
 */
void Motion::generatePosProfile( uint8_t Motor ) { Planners[ Motor ]->generatePosProfile(); }

/**
 * @brief Evaluate position at specified time
 * @param evalTime Time to evaluate (seconds)
 * @param Motor Motor index (0-3)
 * @return Position at specified time
 */
float Motion::evalPos( float evalTime, uint8_t Motor ) { return Planners[ Motor ]->evalPos( evalTime ); }

/**
 * @brief Evaluate velocity at specified time
 * @param evalTime Time to evaluate (seconds)
 * @param Motor Motor index (0-3)
 * @return Velocity at specified time
 */
float Motion::evalVel( float evalTime, uint8_t Motor ) { return Planners[ Motor ]->evalVel( evalTime ); }

/**
 * @brief Evaluate acceleration at specified time
 * @param evalTime Time to evaluate (seconds)
 * @param Motor Motor index (0-3)
 * @return Acceleration at specified time
 */
float Motion::evalAccel( float evalTime, uint8_t Motor ) { return Planners[ Motor ]->evalAccel( evalTime ); }
