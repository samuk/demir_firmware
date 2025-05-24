/**
 * @file PLANNER.h
 * @brief Trajectory planning class declaration using 5th order polynomials
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for PLANNER class which generates smooth trajectory profiles
 *          using 5th order polynomial interpolation for position control with
 *          zero initial and final velocities and accelerations.
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "conf.h"

/**
 * @class PLANNER
 * @brief 5th order polynomial trajectory planner
 * @details Generates smooth position trajectories using 5th order polynomials
 *          with boundary conditions: pos(0) = initial, pos(T) = final,
 *          vel(0) = vel(T) = 0, accel(0) = accel(T) = 0.
 *          Provides real-time trajectory evaluation for position, velocity, and acceleration.
 */
class PLANNER {
   public:
    PLANNER() = default;  ///< Default constructor

    /**
     * @brief Generate 5th order polynomial coefficients for trajectory
     * @details Calculates polynomial coefficients based on initial position,
     *          final position, and trajectory duration with zero boundary conditions
     *          for velocity and acceleration.
     */
    void  generatePosProfile();
    
    /**
     * @brief Evaluate position at specified time
     * @param evalTime Time to evaluate position at (seconds)
     * @return Position value at specified time
     * @details Calculates position using 5th order polynomial: pos(t) = a₀t⁵ + a₁t⁴ + a₂t³ + a₃
     */
    float evalPos( float evalTime );
    
    /**
     * @brief Evaluate velocity at specified time
     * @param evalTime Time to evaluate velocity at (seconds)
     * @return Velocity value at specified time
     * @details Calculates velocity using 4th order polynomial (derivative of position)
     */
    float evalVel( float evalTime );
    
    /**
     * @brief Evaluate acceleration at specified time
     * @param evalTime Time to evaluate acceleration at (seconds)
     * @return Acceleration value at specified time
     * @details Calculates acceleration using 3rd order polynomial (derivative of velocity)
     */
    float evalAccel( float evalTime );

    // Trajectory parameter setters
    /**
     * @brief Set relative position reference for trajectory
     * @param value Relative position change in encoder counts
     */
    void setRelativePositionReference( const int32_t value ) { _relPosRef = value; }
    
    /**
     * @brief Set trajectory start time
     * @param value Start time in milliseconds
     */
    void setInitialTime( const uint32_t value ) { _initialTime = value; }
    
    /**
     * @brief Set trajectory starting position
     * @param value Initial position in encoder counts
     */
    void setInitialPosition( const int32_t value ) { _initialPosition = value; }
    
    /**
     * @brief Set trajectory ending position
     * @param value Final position in encoder counts
     */
    void setFinalPosition( const int32_t value ) { _finalPosition = value; }
    
    /**
     * @brief Set trajectory duration
     * @param value Duration in seconds
     */
    void setTimeInSec( const float value ) { _timeInSec = value; }
    
    /**
     * @brief Set current relative time within trajectory
     * @param value Current time offset in milliseconds
     */
    void setRelativeTime( const float value ) { _relativeTime = value; }
    
    /**
     * @brief Set trajectory execution flag
     * @param value True to enable trajectory execution
     */
    void setPositionModeFired( const bool value ) { _positionModeFired = value; }

    // Trajectory parameter getters
    /**
     * @brief Get relative position reference
     * @return Relative position change in encoder counts
     */
    int32_t  getRelativePositionReference() { return _relPosRef; }
    
    /**
     * @brief Get trajectory start time
     * @return Start time in milliseconds
     */
    uint32_t getInitialTime() { return _initialTime; }
    
    /**
     * @brief Get trajectory starting position
     * @return Initial position in encoder counts
     */
    int32_t  getInitialPosition() { return _initialPosition; }
    
    /**
     * @brief Get trajectory ending position
     * @return Final position in encoder counts
     */
    int32_t  getFinalPosition() { return _finalPosition; }
    
    /**
     * @brief Get trajectory duration
     * @return Duration in seconds
     */
    float    getTimeInSec() { return _timeInSec; }
    
    /**
     * @brief Get current relative time within trajectory
     * @return Current time offset in milliseconds
     */
    float    getRelativeTime() { return _relativeTime; }
    
    /**
     * @brief Get trajectory execution status
     * @return True if trajectory execution is enabled
     */
    bool     getPositionModeFired() { return _positionModeFired; }

   private:
    float _coeff[ 4 ];  ///< Polynomial coefficients [a₀, a₁, a₂, a₃] for 5th order polynomial

    int32_t  _relPosRef = 0;            ///< Relative position reference (encoder counts)
    uint32_t _initialTime;              ///< Trajectory start time (milliseconds)
    int32_t  _initialPosition;          ///< Starting position (encoder counts)
    int32_t  _finalPosition;            ///< Target position (encoder counts)
    float    _timeInSec = 0;            ///< Trajectory duration (seconds)
    float    _relativeTime;             ///< Current execution time offset (milliseconds)
    bool     _positionModeFired = false;///< Trajectory execution enable flag
};

/// @brief Trajectory planner instance for motor 1
extern PLANNER Planner1;

#if MOTORS > 1
/// @brief Trajectory planner instance for motor 2
extern PLANNER Planner2;
#endif
#if MOTORS > 2
/// @brief Trajectory planner instance for motor 3
extern PLANNER Planner3;
#endif
#if MOTORS > 3
/// @brief Trajectory planner instance for motor 4
extern PLANNER Planner4;
#endif

/// @brief Array of pointers to trajectory planner instances
extern PLANNER* Planners[ MOTORS ];