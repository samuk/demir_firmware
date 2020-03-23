#include "controller.h"

controller Controller;

void controller::init() {
    posPID[ 0 ]->setGains( 0.05, 0.01, 0.0 );
    velPID[ 0 ]->setGains( 0.03, 0.05, 0.0 );
#if MOTORS > 1
    posPID[ 1 ]->setGains( 0.05, 0.01, 0.0 );
    velPID[ 1 ]->setGains( 0.03, 0.05, 0.0 );
#endif
#if MOTORS > 2
    posPID[ 2 ]->setGains( 0.05, 0.01, 0.0 );
    velPID[ 2 ]->setGains( 0.03, 0.05, 0.0 );
#endif
#if MOTORS > 3
    posPID[ 3 ]->setGains( 0.05, 0.01, 0.0 );
    velPID[ 4 ]->setGains( 0.03, 0.05, 0.0 );
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

void controller::positionMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) {
            u[ _Motor ]
                = posPID[ _Motor ]->compute( ( float ) ( positionReference[ _Motor ] - currentPosition[ _Motor ] ),
                                             ( float ) deltaT );  // PID hesaplama fonksiyonu çağrılıyor
        }
    }
}

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

void controller::velocityMode() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) {
            u[ _Motor ]
                = velPID[ _Motor ]->compute( ( float ) ( velocityReference[ _Motor ] - currentVelocity[ _Motor ] ),
                                             ( float ) deltaT );  // PID hesaplama fonksiyonu çağrılıyor
        }
    }
}

void controller::profileVelocityMode() {}

void controller::updateCurrentVelocity() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) {
            velocityFilterInput[ _Motor ] = ( float ) ( currentPosition[ _Motor ] - previousPosition[ _Motor ] )
                                            / ( float ) deltaT * 1000.0f;  // encPulse per second
            previousPosition[ _Motor ] = currentPosition[ _Motor ];
            // velocity filtering
            currentVelocity[ _Motor ]
                = alpha * velocityFilterInput[ _Motor ] + ( 1 - alpha ) * previousVelocity[ _Motor ];
            previousVelocity[ _Motor ] = currentVelocity[ _Motor ];
            // currentVel filtre çıkışı
        }
    }
}

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

void controller::updateCurrentPosition() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) this->currentPosition[ _Motor ] = Motor.getPosition( _Motor );
    }
}

void controller::update() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        if ( Motor.isEnabled( _Motor ) ) Motor.setSpeed( u[ _Motor ], _Motor );
    }
}

void controller::enablePosPID( uint8_t Motor ) { posPID[ Motor ]->enable(); }

void controller::disablePosPID( uint8_t Motor ) { posPID[ Motor ]->disable(); }

void controller::enableVelPID( uint8_t Motor ) { velPID[ Motor ]->enable(); }

void controller::disableVelPID( uint8_t Motor ) { velPID[ Motor ]->disable(); }

float controller::getPositionKp( uint8_t Motor ) { return posPID[ Motor ]->getKp(); }
float controller::getPositionKi( uint8_t Motor ) { return posPID[ Motor ]->getKi(); }
float controller::getPositionKd( uint8_t Motor ) { return posPID[ Motor ]->getKd(); }

float controller::getVeloticyKp( uint8_t Motor ) { return velPID[ Motor ]->getKd(); }
float controller::getVeloticyKi( uint8_t Motor ) { return velPID[ Motor ]->getKd(); }
float controller::getVeloticyKd( uint8_t Motor ) { return velPID[ Motor ]->getKd(); }

void controller::setPositionPIDGains( uint8_t Motor, float Kp, float Ki, float Kd ) {
    posPID[ Motor ]->setGains( Kp, Ki, Kd );
}
void controller::setVelocityPIDGains( uint8_t Motor, float Kp, float Ki, float Kd ) {
    velPID[ Motor ]->setGains( Kp, Ki, Kd );
}