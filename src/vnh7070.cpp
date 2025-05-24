/**
 * @file vnh7070.cpp
 * @brief VNH7070 motor driver implementation
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file contains the implementation of the VNH7070 motor driver class
 *          for controlling DC motors with PWM, direction control and current sensing.
 */

#include "vnh7070.h"

/// @brief Motor driver instance for motor 1
vnh7070 motor_1;

#if MOTORS > 1
/// @brief Motor driver instance for motor 2
vnh7070 motor_2;
#endif
#if MOTORS > 2
/// @brief Motor driver instance for motor 3
vnh7070 motor_3;
#endif
#if MOTORS > 3
/// @brief Motor driver instance for motor 4
vnh7070 motor_4;
#endif

/// @brief Array of pointers to motor driver instances
vnh7070* Motors[ MOTORS ] = { &motor_1 };

/**
 * @brief Parameterized constructor for VNH7070 motor driver
 * @param pinINA Input A pin for direction control
 * @param pinINB Input B pin for direction control
 * @param pinPWM PWM output pin for speed control
 * @param pinSEL0 Select pin for current sensing range
 * @param pinCSENSE Current sensing analog input pin
 */
vnh7070::vnh7070( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE )
    : _pinINA( pinINA ),
      _pinINB( pinINB ),
      _pinPWM( pinPWM ),
      _pinSEL0( pinSEL0 ),
      _pinCSENSE( pinCSENSE ) {}

/**
 * @brief Initialize the VNH7070 motor driver with pin assignments
 * @param pinINA Input A pin for direction control
 * @param pinINB Input B pin for direction control  
 * @param pinPWM PWM output pin for speed control
 * @param pinSEL0 Select pin for current sensing range
 * @param pinCSENSE Current sensing analog input pin
 * @details Sets up all control pins as outputs, sensing pin as input,
 *          and configures PWM timer for 16-bit operation
 */
void vnh7070::init( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE ) {
    _pinINA    = pinINA;
    _pinINB    = pinINB;
    _pinPWM    = pinPWM;
    _pinSEL0   = pinSEL0;
    _pinCSENSE = pinCSENSE;

    // Initialize direction control pins
    digitalWrite( _pinINA, LOW );
    pinMode( _pinINA, OUTPUT );

    digitalWrite( _pinINB, LOW );
    pinMode( _pinINB, OUTPUT );

    // Initialize current sensing select pin
    digitalWrite( _pinSEL0, LOW );
    pinMode( _pinSEL0, OUTPUT );

    // Initialize current sensing input
    pinMode( _pinCSENSE, INPUT );
    
    // Setup PWM timer configuration
    setupPFC_PWM();
}

/**
 * @brief Convert control voltage to PWM output with direction control
 * @param uIn Control input voltage/signal (-255 to +255)
 * @details Positive values rotate motor in forward direction,
 *          negative values rotate in reverse direction,
 *          zero stops the motor by setting both direction pins LOW
 */
void vnh7070::uToPWM( float uIn ) {
    // Set PWM duty cycle based on absolute value
    analogWrite16( _pinPWM, abs( uIn ) );
    
    if ( uIn == 0 ) {
        // Brake mode - both inputs LOW
        digitalWrite( _pinINA, LOW );
        digitalWrite( _pinINB, LOW );
    } else if ( uIn > 0 ) {
        // Forward direction
        digitalWrite( _pinINA, _motorDir );
        digitalWrite( _pinINB, !_motorDir );
    } else {
        // Reverse direction
        digitalWrite( _pinINA, !_motorDir );
        digitalWrite( _pinINB, _motorDir );
    }
}

/**
 * @brief Setup Timer1 for Phase and Frequency Correct PWM mode
 * @details Configures Timer1 for 16-bit PWM operation with variable resolution
 *          based on ANALOG_WRITE_RES setting (8, 9, or 10 bits)
 */
void vnh7070::setupPFC_PWM() {
    // Clear timer compare registers
    OCR1A = 0;
    OCR1B = 0;
    
    // Set PWM pins as outputs
    pinMode( _pinPWM, OUTPUT );
    pinMode( 10, OUTPUT );

    // Set TOP value based on resolution
    if ( ANALOG_WRITE_RES == 8 )
        ICR1 = 255;
    else if ( ANALOG_WRITE_RES == 9 )
        ICR1 = 511;
    else if ( ANALOG_WRITE_RES == 10 )
        ICR1 = 1023;

    // Configure Timer1 for Phase and Frequency Correct PWM
    TCCR1A = 0xA0;  // COM1A1, COM1B1 set for non-inverting PWM
    TCCR1B = 0x11;  // WGM13 set, prescaler = 1
}

/**
 * @brief Write 16-bit value to Timer1 compare registers for PWM output
 * @param pinPWM PWM pin number (TIMER1_A_PIN or TIMER1_B_PIN)
 * @param val 16-bit PWM duty cycle value
 * @details Maps PWM pin to corresponding Timer1 output compare register
 */
void vnh7070::analogWrite16( uint8_t pinPWM, uint16_t val ) {
    if ( pinPWM == TIMER1_A_PIN )
        OCR1A = val;
    else if ( pinPWM == TIMER1_B_PIN )
        OCR1B = val;
}