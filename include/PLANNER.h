#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "conf.h"

class PLANNER {
   public:
    PLANNER() = default;

    void  generatePosProfile();
    float evalPos( float evalTime );
    float evalVel( float evalTime );
    float evalAccel( float evalTime );

    void setRelativePositionReference( const int32_t value ) { _relPosRef = value; }
    void setInitialTime( const uint32_t value ) { _initialTime = value; }
    void setInitialPosition( const int32_t value ) { _initialPosition = value; }
    void setFinalPosition( const int32_t value ) { _finalPosition = value; }
    void setTimeInSec( const float value ) { _timeInSec = value; }
    void setRelativeTime( const float value ) { _relativeTime = value; }
    void setPositionModeFired( const bool value ) { _positionModeFired = value; }

    int32_t  getRelativePositionReference() { return _relPosRef; }
    uint32_t getInitialTime() { return _initialTime; }
    int32_t  getInitialPosition() { return _initialPosition; }
    int32_t  getFinalPosition() { return _finalPosition; }
    float    getTimeInSec() { return _timeInSec; }
    float    getRelativeTime() { return _relativeTime; }
    bool     getPositionModeFired() { return _positionModeFired; }

   private:
    float _coeff[ 4 ];

    int32_t  _relPosRef = 0;
    uint32_t _initialTime;
    int32_t  _initialPosition;
    int32_t  _finalPosition;
    float    _timeInSec = 0;
    float    _relativeTime;
    bool     _positionModeFired = false;
};

extern PLANNER Planner1;

#if MOTORS > 1
extern PLANNER Planner2;
#endif
#if MOTORS > 2
extern PLANNER Planner3;
#endif
#if MOTORS > 3
extern PLANNER Planner4;
#endif

extern PLANNER* Planners[ MOTORS ];