/**
 * @file SerialParser.h
 * @brief Serial command parsing class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for SerialParser class which handles parsing of incoming
 *          serial commands with proper synchronization and error handling.
 */

#pragma once

#include <Arduino.h>

#include "conf.h"

/**
 * @def MAX_DATA_NUM
 * @brief Maximum number of data elements in a command
 */
#define MAX_DATA_NUM 5

/**
 * @class SerialParser
 * @brief Serial command parser with synchronization
 * @details Parses floating-point commands from serial port with proper delimiter
 *          handling, buffer overflow protection, and command synchronization.
 *          Supports commands with up to 5 floating-point parameters.
 */
class SerialParser {
   public:
    SerialParser() = default;  ///< Default constructor
    
    /**
     * @brief Parse incoming serial commands from UART
     * @details Reads floating-point values separated by spaces/tabs and terminated
     *          by carriage return or newline. Handles synchronization and prevents
     *          buffer overflow with proper error recovery.
     */
    void readCommands();

    bool    _Status               = 0;           ///< Command parsing status flag
    float   _Data[ MAX_DATA_NUM ] = { 0 };      ///< Parsed command data array
    uint8_t _Index                = 0;          ///< Current data array index
};