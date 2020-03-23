#include "Motion.h"

Motion Planner;

void Motion::enable( uint8_t Motor ) { Planners[ Motor ]->setPositionModeFired( true ); }

void Motion::disable() {
    for ( uint8_t _Motor = 0; _Motor < MOTORS; _Motor++ ) {
        Planners[ _Motor ]->setPositionModeFired( false );
    }
}

void Motion::disable( uint8_t Motor ) { Planners[ Motor ]->setPositionModeFired( true ); }

bool Motion::isEnabled( uint8_t Motor ) { return Planners[ Motor ]->getPositionModeFired(); }

bool Motion::isDisabled( uint8_t Motor ) { return !Planners[ Motor ]->getPositionModeFired(); }

void Motion::setRelativePositionReference( const int32_t value, uint8_t Motor ) {
    Planners[ Motor ]->setRelativePositionReference( value );
}

void Motion::setInitialTime( const uint32_t value, uint8_t Motor ) { Planners[ Motor ]->setInitialTime( value ); }

void Motion::setInitialPosition( const int32_t value, uint8_t Motor ) {
    Planners[ Motor ]->setInitialPosition( value );
}

void Motion::setFinalPosition( const int32_t value, uint8_t Motor ) { Planners[ Motor ]->setFinalPosition( value ); }

void Motion::setTimeInSec( const float value, uint8_t Motor ) { Planners[ Motor ]->setTimeInSec( value ); }

void Motion::setRelativeTime( const float value, uint8_t Motor ) { Planners[ Motor ]->setRelativeTime( value ); }

int32_t Motion::getRelativePositionReference( uint8_t Motor ) {
    return Planners[ Motor ]->getRelativePositionReference();
}

uint32_t Motion::getInitialTime( uint8_t Motor ) { return Planners[ Motor ]->getInitialTime(); }

int32_t Motion::getInitialPosition( uint8_t Motor ) { return Planners[ Motor ]->getInitialPosition(); }

int32_t Motion::getFinalPosition( uint8_t Motor ) { return Planners[ Motor ]->getFinalPosition(); }

float Motion::getTimeInSec( uint8_t Motor ) { return Planners[ Motor ]->getTimeInSec(); }

float Motion::getRelativeTime( uint8_t Motor ) { return Planners[ Motor ]->getRelativeTime(); }

void Motion::generatePosProfile( uint8_t Motor ) { Planners[ Motor ]->generatePosProfile(); }

float Motion::evalPos( float evalTime, uint8_t Motor ) { return Planners[ Motor ]->evalPos( evalTime ); }

float Motion::evalVel( float evalTime, uint8_t Motor ) { return Planners[ Motor ]->evalVel( evalTime ); }

float Motion::evalAccel( float evalTime, uint8_t Motor ) { return Planners[ Motor ]->evalAccel( evalTime ); }
