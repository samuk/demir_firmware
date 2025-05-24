/**
 * @file encoder.h
 * @brief Quadrature encoder interface class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for encoder class which handles quadrature encoder reading
 *          using hardware interrupts for precise position feedback with direction
 *          detection and user-configurable direction mapping.
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "conf.h"

/**
 * @class encoder
 * @brief Quadrature encoder interface with interrupt-based reading
 * @details Implements quadrature encoder position tracking using hardware interrupts
 *          for high-resolution position feedback. Supports direction detection,
 *          position reset, and user-configurable direction mapping for flexibility.
 */
class encoder {
   private:
    uint8_t          _pinENCA    = 0xFF;   ///< Encoder channel A pin
    uint8_t          _pinENCB    = 0xFF;   ///< Encoder channel B pin
    volatile int32_t _encPos     = 0;      ///< Current encoder position
    bool             _userPosDir = false;  ///< User-defined direction flag

   public:
    encoder() = default;  ///< Default constructor
    
    /**
     * @brief Parameterized constructor
     * @param pinENCA Encoder channel A pin
     * @param pinENCB Encoder channel B pin
     */
    encoder( uint8_t pinENCA, uint8_t pinENCB );
    
    /**
     * @brief Initialize encoder with hardware interrupt setup
     * @param pinENCA Encoder channel A pin (must be interrupt-capable)
     * @param pinENCB Encoder channel B pin (must be interrupt-capable)
     */
    void    init( uint8_t pinENCA, uint8_t pinENCB );
    
    /**
     * @brief Get current encoder position with direction compensation
     * @return Current position in encoder counts
     */
    int32_t getPosition();
    
    /**
     * @brief Reset encoder position to zero
     * @details Clears the current position counter
     */
    void    resetPosition() { this->_encPos = 0; }
    
    /**
     * @brief Set user-defined position direction
     * @param dir Direction flag (true/false)
     * @details Allows reversing position sign without hardware changes
     */
    void    setUserPosDir( bool dir ) { this->_userPosDir = dir; }
    
    /**
     * @brief Get user-defined position direction setting
     * @return Current direction flag
     */
    bool    getUserPosDir() { return this->_userPosDir; }
};

/// @brief Encoder instance for motor 1
extern encoder encoder_1;

#if MOTORS > 1
/// @brief Encoder instance for motor 2
extern encoder encoder_2;
#endif
#if MOTORS > 2
/// @brief Encoder instance for motor 3
extern encoder encoder_3;
#endif
#if MOTORS > 3
/// @brief Encoder instance for motor 4
extern encoder encoder_4;
#endif

/// @brief Array of pointers to encoder instances
extern encoder* Encoders[ MOTORS ];