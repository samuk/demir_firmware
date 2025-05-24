/**
 * @file PID.cpp
 * @brief PID controller implementation for motor control
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements a discrete PID controller with anti-windup,
 *          output limiting, and configurable loop rate for precise motor control.
 */

#include "PID.h"

/// @brief Position PID controller instance for motor 1
PID posPID_1;
/// @brief Velocity PID controller instance for motor 1
PID velPID_1;

#if ( MOTORS > 1 )
/// @brief Position PID controller instance for motor 2
PID posPID_2;
/// @brief Velocity PID controller instance for motor 2
PID velPID_2;
#endif

#if ( MOTORS > 2 )
/// @brief Position PID controller instance for motor 3
PID posPID_3;
/// @brief Velocity PID controller instance for motor 3
PID velPID_3;
#endif

#if ( MOTORS > 3 )
/// @brief Position PID controller instance for motor 4
PID posPID_4;
/// @brief Velocity PID controller instance for motor 4
PID velPID_4;
#endif

/// @brief Array of pointers to velocity PID controller instances
PID* velPID[ MOTORS ] = { &velPID_1 };
/// @brief Array of pointers to position PID controller instances
PID* posPID[ MOTORS ] = { &posPID_1 };

/**
 * @brief Default constructor for PID controller
 * @details Initializes all PID parameters to safe default values with
 *          output limits set to ±255 and 3ms loop rate
 */
PID::PID()
    : _enabled( false ),        ///< Controller disabled by default
      _u( 0.0f ),              ///< Zero output initially
      _kP( 0.0f ),             ///< Zero proportional gain
      _kI( 0.0f ),             ///< Zero integral gain
      _kD( 0.0f ),             ///< Zero derivative gain
      _error( 0.0f ),          ///< Zero initial error
      _previousError( 0.0f ),  ///< Zero previous error
      _uP( 0.0f ),             ///< Zero proportional component
      _uI( 0.0f ),             ///< Zero integral component
      _uD( 0.0f ),             ///< Zero derivative component
      _outMax( 255.0f ),       ///< Maximum output value
      _outMin( -255.0f ),      ///< Minimum output value
      _currentTime( 0 ),       ///< Current timestamp
      _oldTime( 0 ),           ///< Previous timestamp
      _deltaT( 0 ),            ///< Time difference
      _loopRate( 3 ) {}        ///< 3ms loop rate default

/**
 * @brief Enable the PID controller
 * @details Sets the controller active state to true
 */
void PID::enable() { _enabled = true; }

/**
 * @brief Disable the PID controller and reset internal states
 * @details Sets controller inactive, clears output and integral term,
 *          and resets previous error to prevent derivative kick
 */
void PID::disable() {
    _enabled       = false;
    _u             = 0.0f;
    _uI            = 0.0f;
    _previousError = 0.0f;
}

/**
 * @brief Compute PID output with automatic timing
 * @param err Error signal (setpoint - measured_value)
 * @return 1 if computation was performed, 0 if skipped due to timing, -1 if disabled
 * @details Performs PID calculation only when enough time has elapsed based
 *          on loop rate setting. Includes anti-windup for integral term.
 */
uint8_t PID::compute( float err ) {
    if ( !getStatus() ) return -1;

    _currentTime = millis();
    _deltaT      = _currentTime - _oldTime;

    if ( _deltaT >= _loopRate ) {
        _oldTime = _currentTime;

        _error = err;

        // Proportional term
        _uP = _kP * _error;

        // Integral term with anti-windup
        _uI += _kI * _error * float( _deltaT ) / 1000.0f;
        _uI = constrain( _uI, _outMin, _outMax );

        // Derivative term
        _uD            = _kD * ( _error - _previousError ) * 1000.0f / float( _deltaT );
        _previousError = _error;

        // Combine all terms and apply output limits
        _u = _uP + _uI + _uD;
        _u = constrain( _u, _outMin, _outMax );

        return 1;
    }
    return 0;
}

/**
 * @brief Compute PID output with specified time step
 * @param err Error signal (setpoint - measured_value)
 * @param deltaT Time step in milliseconds
 * @return PID controller output
 * @details Performs PID calculation with user-specified time step,
 *          useful for fixed-rate control loops
 */
float PID::compute( float err, float deltaT ) {
    if ( !getStatus() ) return 0;

    _error = err;

    // Proportional term
    _uP = _kP * _error;

    // Integral term with anti-windup
    _uI += _kI * _error * deltaT / 1000.0f;
    _uI = constrain( _uI, _outMin, _outMax );

    // Derivative term
    _uD            = _kD * ( _error - _previousError ) * 1000.0f / deltaT;
    _previousError = _error;

    // Combine all terms and apply output limits
    _u = _uP + _uI + _uD;
    _u = constrain( _u, _outMin, _outMax );

    return _u;
}

/**
 * @brief Set proportional gain
 * @param kP Proportional gain value (must be non-negative)
 */
void PID::setKp( const float kP ) {
    if ( kP < 0 ) return;
    _kP = kP;
}

/**
 * @brief Set integral gain
 * @param kI Integral gain value (must be non-negative)
 */
void PID::setKi( const float kI ) {
    if ( kI < 0 ) return;
    _kI = kI;
}

/**
 * @brief Set derivative gain
 * @param kD Derivative gain value (must be non-negative)
 */
void PID::setKd( const float kD ) {
    if ( kD < 0 ) return;
    _kD = kD;
}

/**
 * @brief Set all PID gains simultaneously
 * @param kP Proportional gain (must be non-negative)
 * @param kI Integral gain (must be non-negative)
 * @param kD Derivative gain (must be non-negative)
 * @details All gains must be non-negative, invalid values are ignored
 */
void PID::setGains( const float kP, const float kI, const float kD ) {
    if ( kP < 0 || kI < 0 || kD < 0 ) return;
    _kP = kP;
    _kI = kI;
    _kD = kD;
}

/**
 * @brief Set output limits for the PID controller
 * @param outMin Minimum output value
 * @param outMax Maximum output value
 * @details Only sets limits if outMin < outMax
 */
void PID::setOutMinMax( const float outMin, const float outMax ) {
    if ( outMin < outMax ) {
        _outMin = outMin;
        _outMax = outMax;
    }
}
