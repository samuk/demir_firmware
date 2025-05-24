/**
 * @file PID.h
 * @brief PID controller class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for discrete PID controller implementation with
 *          anti-windup, output limiting, and configurable timing for motor control.
 */

#pragma once

#include <Arduino.h>

#include "conf.h"

/**
 * @class PID
 * @brief Discrete PID controller with anti-windup and output limiting
 * @details Implements a digital PID controller suitable for motor control applications.
 *          Features include automatic timing, integral windup protection, configurable
 *          output limits, and both automatic and manual timing modes.
 */
class PID {
   public:
    /**
     * @brief Default constructor
     * @details Initializes PID controller with safe default values
     */
    PID();

    /**
     * @brief Compute PID output with automatic timing
     * @param err Error signal (setpoint - measured_value)
     * @return Status: 1 if computed, 0 if skipped, -1 if disabled
     */
    uint8_t compute( float err );
    
    /**
     * @brief Compute PID output with specified time step
     * @param err Error signal (setpoint - measured_value)
     * @param deltaT Time step in milliseconds
     * @return PID controller output
     */
    float   compute( float err, float deltaT );

    /**
     * @brief Enable the PID controller
     */
    void enable();
    
    /**
     * @brief Disable the PID controller and reset states
     */
    void disable();
    
    /**
     * @brief Set proportional gain
     * @param kP Proportional gain (must be non-negative)
     */
    void setKp( const float kP );
    
    /**
     * @brief Set integral gain
     * @param kI Integral gain (must be non-negative)
     */
    void setKi( const float kI );
    
    /**
     * @brief Set derivative gain
     * @param kD Derivative gain (must be non-negative)
     */
    void setKd( const float kD );
    
    /**
     * @brief Set all PID gains simultaneously
     * @param kP Proportional gain (default: 0)
     * @param kI Integral gain (default: 0)
     * @param kD Derivative gain (default: 0)
     */
    void setGains( const float kP = 0, const float kI = 0, const float kD = 0 );
    
    /**
     * @brief Set output limits
     * @param outMin Minimum output value
     * @param outMax Maximum output value
     */
    void setOutMinMax( const float outMin, const float outMax );
    
    /**
     * @brief Set control loop rate
     * @param loopRate Loop rate in milliseconds
     */
    void setLoopRate( const uint8_t loopRate ) { this->_loopRate = loopRate; }

    // Getter methods
    float   getU() const { return _u; }                    ///< Get total PID output
    float   getKp() const { return _kP; }                  ///< Get proportional gain
    float   getKi() const { return _kI; }                  ///< Get integral gain
    float   getKd() const { return _kD; }                  ///< Get derivative gain
    float   getUp() const { return _uP; }                  ///< Get proportional component
    float   getUi() const { return _uI; }                  ///< Get integral component
    float   getUd() const { return _uD; }                  ///< Get derivative component
    bool    getStatus() const { return _enabled; }        ///< Get enable status
    uint8_t getLoopRate() const { return _loopRate; }     ///< Get loop rate

   private:
    bool          _enabled;        ///< Controller enable flag
    float         _u;              ///< Total PID output
    float         _kP;             ///< Proportional gain
    float         _kI;             ///< Integral gain
    float         _kD;             ///< Derivative gain
    float         _error;          ///< Current error
    float         _previousError;  ///< Previous error for derivative calculation
    float         _uP;             ///< Proportional component output
    float         _uI;             ///< Integral component output (with anti-windup)
    float         _uD;             ///< Derivative component output
    float         _outMax;         ///< Maximum output limit
    float         _outMin;         ///< Minimum output limit
    unsigned long _currentTime;    ///< Current timestamp
    unsigned long _oldTime;        ///< Previous computation timestamp
    unsigned long _deltaT;         ///< Time difference between computations
    uint8_t       _loopRate;       ///< Control loop rate in milliseconds
};

/// @brief Position PID controller for motor 1
extern PID posPID_1;
/// @brief Velocity PID controller for motor 1
extern PID velPID_1;

#if ( MOTORS >= 2 )
/// @brief Position PID controller for motor 2
extern PID posPID_2;
/// @brief Velocity PID controller for motor 2
extern PID velPID_2;
#endif

#if ( MOTORS >= 3 )
/// @brief Position PID controller for motor 3
extern PID posPID_3;
/// @brief Velocity PID controller for motor 3
extern PID velPID_3;
#endif

#if ( MOTORS >= 4 )
/// @brief Position PID controller for motor 4
extern PID posPID_4;
/// @brief Velocity PID controller for motor 4
extern PID velPID_4;
#endif

/// @brief Array of pointers to velocity PID controllers
extern PID* velPID[ MOTORS ];
/// @brief Array of pointers to position PID controllers
extern PID* posPID[ MOTORS ];