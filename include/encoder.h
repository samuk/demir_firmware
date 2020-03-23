#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "conf.h"

class encoder {
   private:
    uint8_t          _pinENCA    = 0xFF;
    uint8_t          _pinENCB    = 0xFF;
    volatile int32_t _encPos     = 0;
    bool             _userPosDir = false;

   public:
    encoder() = default;
    encoder( uint8_t pinENCA, uint8_t pinENCB );
    void    init( uint8_t pinENCA, uint8_t pinENCB );
    int32_t getPosition();
    void    resetPosition() { this->_encPos = 0; }
    void    setUserPosDir( bool dir ) { this->_userPosDir = dir; }
    bool    getUserPosDir() { return this->_userPosDir; }
};

extern encoder encoder_1;

#if MOTORS > 1
extern encoder encoder_2;
#endif
#if MOTORS > 2
extern encoder encoder_3;
#endif
#if MOTORS > 3
extern encoder encoder_4;
#endif

extern encoder* Encoders[ MOTORS ];