#pragma once

#include <Arduino.h>

#include "Motion.h"
#include "MotorDriver.h"
#include "PID.h"
#include "PLANNER.h"
#include "SerialParser.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "com_def.h"
#include "conf.h"
#include "motor.h"

class controller {
   private:
    float   u[ MOTORS ]                   = { 0 };
    float   uManuel[ MOTORS ]             = { 0 };
    int32_t positionReference[ MOTORS ]   = { 0 };
    int32_t currentPosition[ MOTORS ]     = { 0 };
    int32_t previousPosition[ MOTORS ]    = { 0 };
    float   currentVelocity[ MOTORS ]     = { 0 };
    float   previousVelocity[ MOTORS ]    = { 0 };
    float   velocityFilterInput[ MOTORS ] = { 0 };
    float   velocityReference[ MOTORS ]   = { 0 };
    float   alpha                         = 0.2f;

   public:
    controller() = default;

    void init();
    void zeroOutput();
    void set_uManuel( uint8_t uManuel, uint8_t _Motor );
    void manuelMode();
    void positionMode();
    void profilePositionMode();
    void velocityMode();
    void profileVelocityMode();
    void updateCurrentVelocity();

    void setPositionReference( int32_t ref, uint8_t Motor );
    void updateCurrentPosition();
    void update();

    void enablePosPID( uint8_t Motor );
    void disablePosPID( uint8_t Motor );

    void enableVelPID( uint8_t Motor );
    void disableVelPID( uint8_t Motor );

    float getPositionKp( uint8_t Motor );
    float getPositionKi( uint8_t Motor );
    float getPositionKd( uint8_t Motor );

    float getVeloticyKp( uint8_t Motor );
    float getVeloticyKi( uint8_t Motor );
    float getVeloticyKd( uint8_t Motor );

    void setPositionPIDGains( uint8_t Motor, float Kp, float Ki, float Kd );
    void setVelocityPIDGains( uint8_t Motor, float Kp, float Ki, float Kd );

    unsigned long deltaT      = 0;
    unsigned long currentTime = 0;
    unsigned long oldTime     = 0;
};

extern controller Controller;