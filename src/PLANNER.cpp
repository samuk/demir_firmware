/**
 * @file PLANNER.cpp
 * @brief Trajectory planning implementation using 5th order polynomials
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements the PLANNER class for generating smooth trajectory
 *          profiles using 5th order polynomial interpolation for position control.
 *          Based on trajectory generation principles from SNU OCW materials.
 */

#include "PLANNER.h"

/// @brief Trajectory planner instance for motor 1
PLANNER Planner1;

#if MOTORS > 1
/// @brief Trajectory planner instance for motor 2
PLANNER Planner2;
#endif
#if MOTORS > 2
/// @brief Trajectory planner instance for motor 3
PLANNER Planner3;
#endif
#if MOTORS > 3
/// @brief Trajectory planner instance for motor 4
PLANNER Planner4;
#endif

/// @brief Array of pointers to planner instances
PLANNER* Planners[ MOTORS ] = { &Planner1 };

/**
 * @brief Generate 5th order polynomial coefficients for position profile
 * @details Calculates polynomial coefficients for smooth trajectory from initial
 *          to final position with zero initial and final velocities and accelerations.
 *          Uses boundary conditions: pos(0) = initial, pos(T) = final,
 *          vel(0) = vel(T) = 0, accel(0) = accel(T) = 0
 * @note Based on: http://ocw.snu.ac.kr/sites/default/files/NOTE/Chap07_Trajectory%20generation.pdf
 */
void PLANNER::generatePosProfile() {
    // Calculate 5th order polynomial coefficients
    _coeff[ 0 ] = 12 * ( ( float ) ( _finalPosition - _initialPosition ) ) / ( 2 * pow( _timeInSec, 5 ) );
    _coeff[ 1 ] = -30 * ( ( float ) ( _finalPosition - _initialPosition ) ) / ( 2 * pow( _timeInSec, 4 ) );
    _coeff[ 2 ] = 20 * ( ( float ) ( _finalPosition - _initialPosition ) ) / ( 2 * pow( _timeInSec, 3 ) );
    _coeff[ 3 ] = ( float ) _initialPosition;
}

/**
 * @brief Evaluate position at given time using 5th order polynomial
 * @param evalTime Time value to evaluate position at (seconds)
 * @return Position value at specified time
 * @details Calculates: pos(t) = a₀t⁵ + a₁t⁴ + a₂t³ + a₃
 */
float PLANNER::evalPos( float evalTime ) {
    return ( _coeff[ 0 ] * pow( evalTime, 5 ) + _coeff[ 1 ] * pow( evalTime, 4 ) + _coeff[ 2 ] * pow( evalTime, 3 )
             + _coeff[ 3 ] );
}

/**
 * @brief Evaluate velocity at given time using 4th order polynomial (derivative of position)
 * @param evalTime Time value to evaluate velocity at (seconds)
 * @return Velocity value at specified time
 * @details Calculates: vel(t) = 5a₀t⁴ + 4a₁t³ + 3a₂t²
 */
float PLANNER::evalVel( float evalTime ) {
    return ( 5.0 * _coeff[ 0 ] * pow( evalTime, 4 ) + 4.0 * _coeff[ 1 ] * pow( evalTime, 3 )
             + 3.0 * _coeff[ 2 ] * pow( evalTime, 2 ) );
}

/**
 * @brief Evaluate acceleration at given time using 3rd order polynomial (derivative of velocity)
 * @param evalTime Time value to evaluate acceleration at (seconds)
 * @return Acceleration value at specified time
 * @details Calculates: accel(t) = 20a₀t³ + 12a₁t² + 6a₂t
 */
float PLANNER::evalAccel( float evalTime ) {
    return ( 20.0 * _coeff[ 0 ] * pow( evalTime, 3 ) + 12.0 * _coeff[ 1 ] * pow( evalTime, 2 )
             + 6.0 * _coeff[ 2 ] * evalTime );
}