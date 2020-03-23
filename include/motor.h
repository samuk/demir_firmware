#pragma once

#include "PID.h"
#include "board/DEMIRv1/hardware.h"
#include "conf.h"
#include "encoder.h"
#include "vnh7070.h"

typedef enum : uint8_t { MOTOR_1 = 0, MOTOR_2, MOTOR_3, MOTOR_4 } cfgMotors;

class motor {
   public:
    motor();
    void init();

    void setSpeed( const uint8_t motor_speed, uint8_t Motor );
    // void setIndex( const uint8_t motor_index ) { _Index = motor_index; }

    // uint8_t getSpeed() const { return _Speed; }
    // uint8_t getIndex() const { return _Index; }

    void setDir( bool dir, uint8_t Motor );
    bool getDir( uint8_t Motor );

    void enable( uint8_t Motor );
    void disable( uint8_t Motor );
    bool isEnabled( uint8_t Motor );
    bool isDisabled( uint8_t Motor );

    int32_t getPosition( uint8_t Motor );
    void    resetPosition( uint8_t Motor );
    void    setUserPosDir( bool dir, uint8_t Motor );
    bool    getUserPosDir( uint8_t Motor );

   private:
};

extern motor Motor;
