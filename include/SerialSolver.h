#pragma once

#include <Arduino.h>

#include "Motion.h"
#include "MotorDriver.h"
#include "SerialParser.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "com_def.h"
#include "conf.h"
#include "motor.h"

class SerialSolver : public SerialParser {
   public:
    SerialSolver() = default;

    void proccessCommands();
    void proccessSub1Cmd( uint8_t _Motor );
    void proccessSub2Cmd( uint8_t _Motor );
    void proccessSub3Cmd( uint8_t _Motor );
    void proccessSub4Cmd( uint8_t _Motor );

    bool             driverState                  = DISABLED;
    volatile int32_t posRef                       = 0;
    volatile int32_t currentPosition              = 0;
    volatile int32_t prevPosition                 = 0;
    float            currentVel                   = 0;
    float            prevVel                      = 0;
    float            velFilterInput               = 0;
    float            velRef                       = 0;
    bool             logEnable                    = false;
    uint8_t          logWhat                      = LOG_POSITION;
    bool             sendLogToMatlab              = false;
    uint16_t         logCounter                   = 0;
    uint16_t         logSize                      = LOG_SIZE_MAX;
    uint32_t         logForMatlab[ LOG_SIZE_MAX ] = { 0 };
    bool             longReportPos                = false;
    bool             longReportVel                = false;
    bool             shortReportPos               = false;
    bool             shortReportVel               = false;
    float            totalCurrent                 = 0;
    float            u                            = 0;
    float            uManual                      = 0;
    uint16_t         printingPeriod               = 1000;
    uint8_t          rs485id                      = 0;
};

extern SerialSolver Solver;