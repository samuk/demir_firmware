/**
 * @file SerialSolver.h
 * @brief Serial command processing and execution class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for SerialSolver class which handles parsing and execution
 *          of serial commands for motor control, PID tuning, system monitoring,
 *          and data logging functionality.
 */

#pragma once

#include <Arduino.h>

#include "Motion.h"
#include "MotorDriver.h"
#include "SerialParser.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "com_def.h"
#include "conf.h"
#include "motor.h"

/**
 * @class SerialSolver
 * @brief Serial command processor and system coordinator
 * @details Inherits from SerialParser to handle command parsing and implements
 *          command execution for motor control, system configuration, and data logging.
 *          Provides interface between serial communication and motor control subsystems.
 */
class SerialSolver : public SerialParser {
   public:
    SerialSolver() = default;  ///< Default constructor

    /**
     * @brief Main command processing entry point
     * @details Processes parsed commands and routes them to appropriate handlers
     */
    void proccessCommands();
    
    /**
     * @brief Process commands with no parameters
     * @param _Motor Motor index to operate on
     */
    void proccessSub1Cmd( uint8_t _Motor );
    
    /**
     * @brief Process commands with one parameter
     * @param _Motor Motor index to operate on
     */
    void proccessSub2Cmd( uint8_t _Motor );
    
    /**
     * @brief Process commands with two parameters
     * @param _Motor Motor index to operate on
     */
    void proccessSub3Cmd( uint8_t _Motor );
    
    /**
     * @brief Process commands with three parameters
     * @param _Motor Motor index to operate on
     */
    void proccessSub4Cmd( uint8_t _Motor );

    bool             driverState                  = DISABLED;     ///< Driver enable/disable state
    volatile int32_t posRef                       = 0;           ///< Position reference value
    volatile int32_t currentPosition              = 0;           ///< Current motor position
    volatile int32_t prevPosition                 = 0;           ///< Previous motor position
    float            currentVel                   = 0;           ///< Current velocity estimate
    float            prevVel                      = 0;           ///< Previous velocity value
    float            velFilterInput               = 0;           ///< Raw velocity calculation input
    float            velRef                       = 0;           ///< Velocity reference value
    bool             logEnable                    = false;       ///< Data logging enable flag
    uint8_t          logWhat                      = LOG_POSITION;///< What data to log (position/current)
    bool             sendLogToMatlab              = false;       ///< Flag to send logged data
    uint16_t         logCounter                   = 0;           ///< Current log buffer index
    uint16_t         logSize                      = LOG_SIZE_MAX;///< Size of log buffer
    uint32_t         logForMatlab[ LOG_SIZE_MAX ] = { 0 };      ///< Data logging buffer
    bool             longReportPos                = false;       ///< Enable detailed position reporting
    bool             longReportVel                = false;       ///< Enable detailed velocity reporting
    bool             shortReportPos               = false;       ///< Enable short position reporting
    bool             shortReportVel               = false;       ///< Enable short velocity reporting
    float            totalCurrent                 = 0;           ///< Total current measurement
    float            u                            = 0;           ///< Control output signal
    float            uManual                      = 0;           ///< Manual control signal
    uint16_t         printingPeriod               = 1000;        ///< Reporting period in milliseconds
    uint8_t          rs485id                      = 0;           ///< RS485 device ID
};

/// @brief Global SerialSolver instance
extern SerialSolver Solver;