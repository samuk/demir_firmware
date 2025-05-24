/**
 * @file SerialParser.cpp
 * @brief Serial command parsing implementation
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements the SerialParser class which handles parsing
 *          of incoming serial commands with proper synchronization and error handling.
 */

#include "SerialParser.h"

/**
 * @brief Parse incoming serial commands from UART
 * @details Reads floating-point values from serial port separated by spaces or tabs,
 *          terminated by carriage return or newline. Handles synchronization
 *          and prevents buffer overflow.
 */
void SerialParser::readCommands() {
    static int8_t _correctionFlag = -1; ///< Synchronization flag for command parsing
    static int    dummy;                ///< Temporary variable for delimiter storage

    while ( Serial.available() > 0 )  // Check if data available in serial buffer
    {
        // Initial synchronization delay
        if ( _correctionFlag == -1 ) delayMicroseconds( 1000 );
        _correctionFlag = 1;
        
        if ( _Index < 5 ) {
            // Parse floating point value and store in data array
            _Data[ _Index++ ] = Serial.parseFloat();
            dummy             = Serial.read();

            // Check for command termination characters
            if ( dummy == '\r' || dummy == '\n' ) {
                _correctionFlag = 2;
                if ( Serial.available() > 0 ) {
                    Serial.read();  // Consume remaining newline
                }
            }
        } else {
            // Buffer overflow protection
            _correctionFlag = 2;
            Serial.parseFloat();  // Discard excess data
        }
    }

    // Set parsing status based on completion state
    if ( _correctionFlag == ( int8_t ) 1 ) {
        _Data[ --_Index ] = -1;  // Mark incomplete command
        _correctionFlag   = -1;
        _Status           = 1;
    } else if ( _correctionFlag == ( int8_t ) 2 ) {
        _correctionFlag = -1;
        _Status         = 1;      // Command complete
    }
}
