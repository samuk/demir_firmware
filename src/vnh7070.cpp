#include "vnh7070.h"
vnh7070 motor_1;

#if MOTORS > 1
vnh7070 motor_2;
#endif
#if MOTORS > 2
vnh7070 motor_3;
#endif
#if MOTORS > 3
vnh7070 motor_4;
#endif

vnh7070* Motors[ MOTORS ] = { &motor_1 };

vnh7070::vnh7070( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE )
    : _pinINA( pinINA ),
      _pinINB( pinINB ),
      _pinPWM( pinPWM ),
      _pinSEL0( pinSEL0 ),
      _pinCSENSE( pinCSENSE ) {}

void vnh7070::init( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE ) {
    _pinINA    = pinINA;
    _pinINB    = pinINB;
    _pinPWM    = pinPWM;
    _pinSEL0   = pinSEL0;
    _pinCSENSE = pinCSENSE;

    digitalWrite( _pinINA, LOW );
    pinMode( _pinINA, OUTPUT );

    digitalWrite( _pinINB, LOW );
    pinMode( _pinINB, OUTPUT );

    digitalWrite( _pinSEL0, LOW );
    pinMode( _pinSEL0, OUTPUT );

    pinMode( _pinCSENSE, INPUT );
    setupPFC_PWM();
}

void vnh7070::uToPWM( float uIn ) {
    analogWrite16( _pinPWM, abs( uIn ) );
    if ( uIn == 0 ) {
        digitalWrite( _pinINA, LOW );
        digitalWrite( _pinINB, LOW );
    } else if ( uIn > 0 ) {
        digitalWrite( _pinINA, _motorDir );
        digitalWrite( _pinINB, !_motorDir );
    } else {
        digitalWrite( _pinINA, !_motorDir );
        digitalWrite( _pinINB, _motorDir );
    }
}

void vnh7070::setupPFC_PWM() {
    OCR1A = 0;
    OCR1B = 0;
    pinMode( _pinPWM, OUTPUT );
    pinMode( 10, OUTPUT );

    if ( ANALOG_WRITE_RES == 8 )
        ICR1 = 255;
    else if ( ANALOG_WRITE_RES == 9 )
        ICR1 = 511;
    else if ( ANALOG_WRITE_RES == 10 )
        ICR1 = 1023;

    TCCR1A = 0xA0;
    TCCR1B = 0x11;
}

void vnh7070::analogWrite16( uint8_t pinPWM, uint16_t val ) {
    if ( pinPWM == TIMER1_A_PIN )
        OCR1A = val;
    else if ( pinPWM == TIMER1_B_PIN )
        OCR1B = val;
}