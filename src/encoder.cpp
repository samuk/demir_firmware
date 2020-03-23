#include "encoder.h"

encoder encoder_1;

#if MOTORS > 1
encoder encoder_2;
#endif
#if MOTORS > 2
encoder encoder_3;
#endif
#if MOTORS > 3
encoder encoder_4;
#endif

encoder* Encoders[ MOTORS ] = { &encoder_1 };

static volatile int32_t encPos = 0;

encoder::encoder( uint8_t pinENCA, uint8_t pinENCB ) : _pinENCA( pinENCA ), _pinENCB( pinENCB ) {}

void encoder::init( uint8_t pinENCA, uint8_t pinENCB ) {
    _pinENCA = pinENCA;
    _pinENCB = pinENCB;

    pinMode( _pinENCA, INPUT_PULLUP );
    pinMode( _pinENCB, INPUT_PULLUP );

    static auto isrPtrA
        = []() { encPos = encPos + 1 - ( ( ( PIND & B00001000 ) >> 2 ) ^ ( ( PIND & B00000100 ) >> 1 ) ); };

    static auto isrPtrB
        = []() { encPos = encPos - 1 + ( ( ( PIND & B00001000 ) >> 2 ) ^ ( ( PIND & B00000100 ) >> 1 ) ); };

    attachInterrupt( digitalPinToInterrupt( _pinENCA ), isrPtrA, CHANGE );
    attachInterrupt( digitalPinToInterrupt( _pinENCB ), isrPtrB, CHANGE );
}

int32_t encoder::getPosition() {
    this->_encPos = encPos;
    return ( getUserPosDir() ? this->_encPos : ( 0 - this->_encPos ) );
}