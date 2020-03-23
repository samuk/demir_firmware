#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "board/DEMIRv1/hardware.h"
#include "conf.h"

class vnh7070 {
   private:
    bool _motorDir = false;
    bool _status   = false;

    uint8_t _pinINA    = 0xFF;
    uint8_t _pinINB    = 0xFF;
    uint8_t _pinPWM    = 0xFF;
    uint8_t _pinSEL0   = 0xFF;
    uint8_t _pinCSENSE = 0xFF;

    FORCE_INLINE void setupPFC_PWM();
    FORCE_INLINE void analogWrite16( uint8_t pinPWM, uint16_t val );

   public:
    vnh7070() = default;
    vnh7070( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE );
    void init( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE );
    void uToPWM( float uIn );
    void setMotorDir( bool dir ) { this->_motorDir = dir; }
    bool getMotorDir() { return this->_motorDir; }
    void setStatus( bool status ) { this->_status = status; }
    bool getStatus() { return _status; }
};

extern vnh7070 motor_1;

#if MOTORS > 1
extern vnh7070 motor_2;
#endif
#if MOTORS > 2
extern vnh7070 motor_3;
#endif
#if MOTORS > 3
extern vnh7070 motor_4;
#endif

extern vnh7070* Motors[ MOTORS ];
