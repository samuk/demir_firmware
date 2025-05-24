/**
 * @file controller.cpp
 * @brief Main control system coordination implementation
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements the controller class which coordinates PID controllers,
 *          sensor feedback, and different control modes for precise motor control.
 */

#include "controller.h"

/// @brief Global controller instance
controller Controller;

/**
 * @brief Initialize control system with default PID gains
 * @details Sets up initial PID gains for position and velocity controllers
 *          for all configured motors. Default gains provide stable operation
 *          but may require tuning for specific applications.
 */
void controller::init() {
    // Initialize PID gains for motor 1
    posPID[ 0 ]->setGains( 0.05, 0.01, 0.0 );  ///< Position PID: Kp=0.05, Ki=0.01, Kd=0.0
    velPID[ 0 ]->setGains( 0.03, 0.05, 0.0 );  ///< Velocity PID: Kp=0.03, Ki=0.05, Kd=0.0
#if MOTORS > 1
    // Initialize PID gains for motor 2
    posPID[ 1 ]->setGains( 0.05, 0.01, 0.0 );
    velPID[ 1 ]->setGains( 0.03, 0.05, 0.0 );
#endif
#if MOTORS > 2
    // Initialize PID gains for motor 3
    posPID[ 2 ]->setGains( 0.05, 0.01, 0.0 );
    velPID[ 2 ]->setGains( 0.03, 0.05, 0.0 );
#endif
#if MOTORS > 3
    // Initialize PID gains for motor 4 (Note: potential array index error)
    posPID[ 3 ]->setGains( 0.05, 0.01, 0.0 );
    velPID[ 4 ]->setGains( 0.03, 0.05, 0.0 );  ///< Index should be 3, not 4
#endif
}

void controller::zeroOutput() {
    for ( uint8_t motor = 0; motor < MOTORS; motor++ ) {
        u[ motor ]       = 0;
        uManuel[ motor ] = 0;
    }
}

void controller::set_uManuel( uint8_t uManuel, uint8_t _Motor ) { u[ _Motor ] = uManuel; }

void controller::manuelMode() {
    for ( uint8_t motor = 0; motor < MOTORS; motor++ ) {
        u[ motor ] = uManuel[ motor ];
    }
}

/**
 * @brief Execute position control mode
 * @details Runs position PID controllers for all enabled motors using
 *          current position feedback and position references
 */
void controller::positionMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) {
            // Calculate position error and run PID controller
            u[ _Motor ]
                = posPID[ _Motor ]->compute( ( float ) ( positionReference[ _Motor ] - currentPosition[ _Motor ] ),
                                             ( float ) deltaT );  // PID hesaplama fonksiyonu çağrılıyor
        }
    }
}

/**
 * @brief Execute profile position control mode
 * @details Runs trajectory-based position control for all enabled motors.
 *          Computes desired position from motion planner output and
 *          runs position PID controller.
 */
void controller::profilePositionMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) {
            if ( Planner.isEnabled( _Motor ) )  // tetiklenmiş mi?
            {
                Planner.setRelativeTime( ( ( float ) currentTime - ( float ) Planner.getInitialTime( _Motor ) ),
                                         _Motor );  // relativeTime
                if ( Planner.getRelativeTime( _Motor ) < Planner.getTimeInSec( _Motor ) * 1000 ) {
                    positionReference[ _Motor ] = Planner1.evalPos( Planner.getRelativeTime( _Motor ) / 1000 );

                } else {                                                               // profil bitmiş demektir
                    Planner.disable( _Motor );                                         // tetiği durdur
                    positionReference[ _Motor ] = Planner.getFinalPosition( _Motor );  // :) çakallık
                    Serial.println( F( "!" ) );  // bittiğine dair seri porttan ! karakteri gönderir
                }
            }
            u[ _Motor ]
                = posPID[ _Motor ]->compute( ( float ) ( positionReference[ _Motor ] - currentPosition[ _Motor ] ),
                                             ( float ) deltaT );  // PID hesaplama fonksiyonu çağrılıyor}
        }
    }
}

/**
 * @brief Execute velocity control mode
 * @details Runs velocity PID controllers for all enabled motors using
 *          current velocity feedback and velocity references
 */
void controller::velocityMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) {
            // Calculate velocity error and run PID controller
            u[ _Motor ]
                = velPID[ _Motor ]->compute( ( float ) ( velocityReference[ _Motor ] - currentVelocity[ _Motor ] ),
                                             ( float ) deltaT );
        }
    }
}

/**
 * @brief Execute profile velocity control mode
 * @details Placeholder for trajectory-based velocity control (not implemented)
 */
void controller::profileVelocityMode() {}

/**
 * @brief Update current velocity estimation using low-pass filtering
 * @details Calculates velocity from position difference and applies exponential
 *          moving average filter to reduce noise. Filter coefficient alpha
 *          determines the balance between responsiveness and noise reduction.
 */
void controller::updateCurrentVelocity() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) {
            // Calculate raw velocity from position difference
            velocityFilterInput[ _Motor ] = ( float ) ( currentPosition[ _Motor ] - previousPosition[ _Motor ] )
                                            / ( float ) deltaT * 1000.0f;  ///< encoder pulses per second
            previousPosition[ _Motor ] = currentPosition[ _Motor ];
            
            // Apply exponential moving average filter
            currentVelocity[ _Motor ]
                = alpha * velocityFilterInput[ _Motor ] + ( 1 - alpha ) * previousVelocity[ _Motor ];
            previousVelocity[ _Motor ] = currentVelocity[ _Motor ];
        }
    }
}

/**
 * @brief Set position reference for specified motor
 * @param ref Position reference in encoder counts
 * @param Motor Motor index (0-3)
 * @details Sets the target position for position control mode
 */
void controller::setPositionReference( int32_t ref, uint8_t Motor ) {
    if ( Motor == MOTOR_1 ) this->positionReference[ MOTOR_1 ] = ref;
#if ( MOTORS > 1 )
    else if ( Motor == MOTOR_2 )
        this->positionReference[ MOTOR_2 ] = ref;
#endif
#if ( MOTORS > 2 )
    else if ( Motor == MOTOR_3 )
        this->positionReference[ MOTOR_3 ] = ref;
#endif
#if ( MOTORS > 3 )
    else if ( Motor == MOTOR_4 )
        this->positionReference[ MOTOR_4 ] = ref;
#endif
}

/**
 * @brief Update current position readings from encoders
 * @details Reads encoder positions for all enabled motors and stores
 *          them in the current position array
 */
void controller::updateCurrentPosition() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) this->currentPosition[ _Motor ] = Motor.getPosition( _Motor );
    }
}

/**
 * @brief Apply control outputs to motors
 * @details Sends computed control signals to motor drivers for all enabled motors
 */
void controller::update() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) Motor.setSpeed( u[ _Motor ], _Motor );
    }
}

/**
 * @brief Enable position PID controller for specified motor
 * @param Motor Motor index (0-3)
 */
void controller::enablePosPID( uint8_t Motor ) { posPID[ Motor ]->enable(); }

/**
 * @brief Disable position PID controller for specified motor
 * @param Motor Motor index (0-3)
 */
void controller::disablePosPID( uint8_t Motor ) { posPID[ Motor ]->disable(); }

/**
 * @brief Enable velocity PID controller for specified motor
 * @param Motor Motor index (0-3)
 */
void controller::enableVelPID( uint8_t Motor ) { velPID[ Motor ]->enable(); }

/**
 * @brief Disable velocity PID controller for specified motor
 * @param Motor Motor index (0-3)
 */
void controller::disableVelPID( uint8_t Motor ) { velPID[ Motor ]->disable(); }

/**
 * @brief Get position PID proportional gain
 * @param Motor Motor index (0-3)
 * @return Proportional gain value
 */
float controller::getPositionKp( uint8_t Motor ) { return posPID[ Motor ]->getKp(); }

/**
 * @brief Get position PID integral gain
 * @param Motor Motor index (0-3)
 * @return Integral gain value
 */
float controller::getPositionKi( uint8_t Motor ) { return posPID[ Motor ]->getKi(); }

/**
 * @brief Get position PID derivative gain
 * @param Motor Motor index (0-3)
 * @return Derivative gain value
 */
float controller::getPositionKd( uint8_t Motor ) { return posPID[ Motor ]->getKd(); }

/**
 * @brief Get velocity PID proportional gain
 * @param Motor Motor index (0-3)
 * @return Proportional gain value
 * @note Function returns getKd() instead of getKp() - potential bug
 */
float controller::getVeloticyKp( uint8_t Motor ) { return velPID[ Motor ]->getKd(); }

/**
 * @brief Get velocity PID integral gain
 * @param Motor Motor index (0-3)
 * @return Integral gain value
 * @note Function returns getKd() instead of getKi() - potential bug
 */
float controller::getVeloticyKi( uint8_t Motor ) { return velPID[ Motor ]->getKd(); }

/**
 * @brief Get velocity PID derivative gain
 * @param Motor Motor index (0-3)
 * @return Derivative gain value
 */
float controller::getVeloticyKd( uint8_t Motor ) { return velPID[ Motor ]->getKd(); }

/**
 * @brief Set position PID gains for specified motor
 * @param Motor Motor index (0-3)
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 */
void controller::setPositionPIDGains( uint8_t Motor, float Kp, float Ki, float Kd ) {
    posPID[ Motor ]->setGains( Kp, Ki, Kd );
}

/**
 * @brief Set velocity PID gains for specified motor
 * @param Motor Motor index (0-3)
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 */
void controller::setVelocityPIDGains( uint8_t Motor, float Kp, float Ki, float Kd ) {
    velPID[ Motor ]->setGains( Kp, Ki, Kd );
}