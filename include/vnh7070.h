/**
 * @file vnh7070.h
 * @brief VNH7070 motor driver class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for VNH7070 H-bridge motor driver IC interface.
 *          Provides PWM speed control, direction control, and current sensing.
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "board/DEMIRv1/hardware.h"
#include "conf.h"

/**
 * @class vnh7070
 * @brief VNH7070 H-bridge motor driver interface class
 * @details Provides control interface for VNH7070 motor driver IC including
 *          PWM speed control, direction control, current sensing, and status monitoring.
 */
class vnh7070 {
   private:
    bool _motorDir = false;  ///< Motor direction flag
    bool _status   = false;  ///< Driver enable status

    uint8_t _pinINA    = 0xFF;  ///< Input A pin for direction control
    uint8_t _pinINB    = 0xFF;  ///< Input B pin for direction control
    uint8_t _pinPWM    = 0xFF;  ///< PWM output pin for speed control
    uint8_t _pinSEL0   = 0xFF;  ///< Select pin for current sensing range
    uint8_t _pinCSENSE = 0xFF;  ///< Current sensing analog input pin

    /**
     * @brief Setup Timer1 for Phase and Frequency Correct PWM
     * @details Configures 16-bit PWM with variable resolution
     */
    FORCE_INLINE void setupPFC_PWM();
    
    /**
     * @brief Write 16-bit value to Timer1 compare registers
     * @param pinPWM PWM pin number
     * @param val 16-bit PWM duty cycle value
     */
    FORCE_INLINE void analogWrite16( uint8_t pinPWM, uint16_t val );

   public:
    vnh7070() = default;  ///< Default constructor
    
    /**
     * @brief Parameterized constructor
     * @param pinINA Input A pin for direction control
     * @param pinINB Input B pin for direction control
     * @param pinPWM PWM output pin for speed control
     * @param pinSEL0 Select pin for current sensing range
     * @param pinCSENSE Current sensing analog input pin
     */
    vnh7070( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE );
    
    /**
     * @brief Initialize motor driver with pin assignments
     * @param pinINA Input A pin for direction control
     * @param pinINB Input B pin for direction control
     * @param pinPWM PWM output pin for speed control
     * @param pinSEL0 Select pin for current sensing range
     * @param pinCSENSE Current sensing analog input pin
     */
    void init( uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinSEL0, uint8_t pinCSENSE );
    
    /**
     * @brief Convert control voltage to PWM output with direction control
     * @param uIn Control input voltage/signal (-255 to +255)
     */
    void uToPWM( float uIn );
    
    /**
     * @brief Set motor rotation direction
     * @param dir Direction flag (true/false)
     */
    void setMotorDir( bool dir ) { this->_motorDir = dir; }
    
    /**
     * @brief Get current motor direction setting
     * @return Current direction flag
     */
    bool getMotorDir() { return this->_motorDir; }
    
    /**
     * @brief Set driver enable status
     * @param status Enable flag (true/false)
     */
    void setStatus( bool status ) { this->_status = status; }
    
    /**
     * @brief Get driver enable status
     * @return Current enable status
     */
    bool getStatus() { return _status; }
};

/// @brief Motor driver instance for motor 1
extern vnh7070 motor_1;

#if MOTORS > 1
/// @brief Motor driver instance for motor 2
extern vnh7070 motor_2;
#endif
#if MOTORS > 2
/// @brief Motor driver instance for motor 3
extern vnh7070 motor_3;
#endif
#if MOTORS > 3
/// @brief Motor driver instance for motor 4
extern vnh7070 motor_4;
#endif

/// @brief Array of pointers to motor driver instances
extern vnh7070* Motors[ MOTORS ];
