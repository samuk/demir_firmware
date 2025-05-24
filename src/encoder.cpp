/**
 * @file encoder.cpp
 * @brief Quadrature encoder interface implementation
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements quadrature encoder reading using hardware interrupts
 *          for precise position feedback with direction detection and user-configurable
 *          direction mapping.
 */

#include "encoder.h"

/// @brief Encoder instance for motor 1
encoder encoder_1;

#if MOTORS > 1
/// @brief Encoder instance for motor 2
encoder encoder_2;
#endif
#if MOTORS > 2
/// @brief Encoder instance for motor 3
encoder encoder_3;
#endif
#if MOTORS > 3
/// @brief Encoder instance for motor 4
encoder encoder_4;
#endif

/// @brief Array of pointers to encoder instances
encoder* Encoders[ MOTORS ] = { &encoder_1 };

/// @brief Global encoder position counter (volatile for ISR access)
static volatile int32_t encPos = 0;

/**
 * @brief Parameterized constructor for encoder
 * @param pinENCA Encoder channel A pin
 * @param pinENCB Encoder channel B pin
 */
encoder::encoder( uint8_t pinENCA, uint8_t pinENCB ) : _pinENCA( pinENCA ), _pinENCB( pinENCB ) {}

/**
 * @brief Initialize encoder with hardware interrupt setup
 * @param pinENCA Encoder channel A pin (must be interrupt-capable)
 * @param pinENCB Encoder channel B pin (must be interrupt-capable)
 * @details Sets up quadrature encoder reading using hardware interrupts on both
 *          channels for 4x resolution. Uses direct port manipulation for fast
 *          direction detection within interrupt service routines.
 */
void encoder::init( uint8_t pinENCA, uint8_t pinENCB ) {
    _pinENCA = pinENCA;
    _pinENCB = pinENCB;

    // Configure encoder pins with internal pull-ups
    pinMode( _pinENCA, INPUT_PULLUP );
    pinMode( _pinENCB, INPUT_PULLUP );

    /**
     * @brief Interrupt service routine for encoder channel A
     * @details Increments or decrements position based on channel B state
     *          Uses direct port reading (PIND) for fast execution
     */
    static auto isrPtrA
        = []() { encPos = encPos + 1 - ( ( ( PIND & B00001000 ) >> 2 ) ^ ( ( PIND & B00000100 ) >> 1 ) ); };

    /**
     * @brief Interrupt service routine for encoder channel B
     * @details Increments or decrements position based on channel A state
     *          Uses direct port reading (PIND) for fast execution
     */
    static auto isrPtrB
        = []() { encPos = encPos - 1 + ( ( ( PIND & B00001000 ) >> 2 ) ^ ( ( PIND & B00000100 ) >> 1 ) ); };

    // Attach interrupts to both encoder channels for 4x resolution
    attachInterrupt( digitalPinToInterrupt( _pinENCA ), isrPtrA, CHANGE );
    attachInterrupt( digitalPinToInterrupt( _pinENCB ), isrPtrB, CHANGE );
}

/**
 * @brief Get current encoder position with direction compensation
 * @return Current position in encoder counts
 * @details Returns the current encoder position accounting for user-defined
 *          direction setting. Positive direction can be reversed if needed
 *          without changing hardware connections.
 */
int32_t encoder::getPosition() {
    this->_encPos = encPos;
    return ( getUserPosDir() ? this->_encPos : ( 0 - this->_encPos ) );
}