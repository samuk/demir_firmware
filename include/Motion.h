#pragma once

#include "PID.h"
#include "PLANNER.h"
#include "board/DEMIRv1/hardware.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "conf.h"
#include "controller.h"
#include "motor.h"

class Motion {
   public:
    Motion() = default;
    void init();

    void enable( uint8_t Motor );
    void disable();
    void disable( uint8_t Motor );
    bool isEnabled( uint8_t Motor );
    bool isDisabled( uint8_t Motor );

    void setRelativePositionReference( const int32_t value, uint8_t Motor );
    void setInitialTime( const uint32_t value, uint8_t Motor );
    void setInitialPosition( const int32_t value, uint8_t Motor );
    void setFinalPosition( const int32_t value, uint8_t Motor );
    void setTimeInSec( const float value, uint8_t Motor );
    void setRelativeTime( const float value, uint8_t Motor );
    void setPositionModeFired( const bool value, uint8_t Motor );

    int32_t  getRelativePositionReference( uint8_t Motor );
    uint32_t getInitialTime( uint8_t Motor );
    int32_t  getInitialPosition( uint8_t Motor );
    int32_t  getFinalPosition( uint8_t Motor );
    float    getTimeInSec( uint8_t Motor );
    float    getRelativeTime( uint8_t Motor );
    bool     getPositionModeFired( uint8_t Motor );

    void  generatePosProfile( uint8_t Motor );
    float evalPos( float evalTime, uint8_t Motor );
    float evalVel( float evalTime, uint8_t Motor );
    float evalAccel( float evalTime, uint8_t Motor );
};

extern Motion Planner;