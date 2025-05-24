/**
 * @file SerialSolver.cpp
 * @brief Serial command processing and execution implementation
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details This file implements the SerialSolver class which handles parsing and
 *          execution of serial commands for motor control, PID tuning, and system monitoring.
 */

#include "SerialSolver.h"

/// @brief Global SerialSolver instance
SerialSolver Solver;

/**
 * @brief Lambda function for writing multi-byte data to serial port
 * @details Writes data bytes in big-endian format (MSB first)
 */
auto writeBytes = [ & ]( auto data, auto size ) {
    for ( size_t i = size; i > 0; i-- ) {
        Serial.write( data );
        data = data >> 8;
    }
};

/**
 * @brief Structure to hold parsed command and parameters
 */
struct Commands {
    uint8_t command     = 0;    ///< Command identifier
    float   params[ 3 ] = { 0 };///< Command parameters array
} _cmd;

/**
 * @brief Union for converting between float and hexadecimal representation
 */
union float2hex {
    uint32_t Hex;   ///< 32-bit hexadecimal representation
    float    Float; ///< Floating point representation
} _float2bits;

/**
 * @brief Main command processing function
 * @details Reads commands from serial port and executes appropriate motor control functions
 *          based on command type and number of parameters received
 */
void SerialSolver::proccessCommands() {
    // Read incoming serial commands
    readCommands();
    
    if ( !_Status ) {
        _Index  = 0;
        _Status = 0;
        return;
    };

    // Validate motor index (0-3)
    if ( !( ( uint8_t ) _Data[ 0 ] >= 0 && ( uint8_t ) _Data[ 0 ] <= 3 ) ) {
        _Index  = 0;
        _Status = 0;
        return;
    }

    // Execute command based on number of parameters
    switch ( _Index ) {
        case 2: {
            _cmd.command = ( uint8_t ) _Data[ 1 ];
            proccessSub1Cmd( ( uint8_t ) _Data[ 0 ] );
            break;
        }
        case 3: {
            _cmd.command     = ( uint8_t ) _Data[ 1 ];
            _cmd.params[ 0 ] = _Data[ 2 ];
            proccessSub2Cmd( ( uint8_t ) _Data[ 0 ] );
            break;
        }
        case 4: {
            _cmd.command     = ( uint8_t ) _Data[ 1 ];
            _cmd.params[ 0 ] = _Data[ 2 ];
            _cmd.params[ 1 ] = _Data[ 3 ];
            proccessSub3Cmd( ( uint8_t ) _Data[ 0 ] );
            break;
        }
        case 5: {
            _cmd.command     = ( uint8_t ) _Data[ 1 ];
            _cmd.params[ 0 ] = _Data[ 2 ];
            _cmd.params[ 1 ] = _Data[ 3 ];
            _cmd.params[ 2 ] = _Data[ 4 ];
            proccessSub4Cmd( ( uint8_t ) _Data[ 0 ] );
            break;
        }
    }
    _Index  = 0;
    _Status = 0;
}

/**
 * @brief Process commands with no parameters (single parameter commands)
 * @param _Motor Motor index to operate on
 * @details Handles system status commands, reporting configuration,
 *          position/velocity queries, and system configuration commands
 */
void SerialSolver::proccessSub1Cmd( uint8_t _Motor ) {
    switch ( _cmd.command ) {
        case COM_PRINT_LONG_VEL_REPORT: {
            // Enable detailed velocity reporting
            longReportPos  = false;
            longReportVel  = true;
            shortReportPos = false;
            shortReportVel = false;
        } break;
        case COM_PRINT_SHORT_VEL_REPORT: {
            // Enable short velocity reporting
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = true;
        } break;
        case COM_PRINT_LONG_POS_REPORT: {
            // Enable detailed position reporting
            longReportPos  = true;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = false;
        } break;
        case COM_PRINT_SHORT_POS_REPORT: {
            // Enable short position reporting
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = true;
            shortReportVel = false;
        } break;
        case COM_PRINT_NO_REPORT: {
            // Disable all reporting
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = false;
        } break;
        case COM_SEND_DATA_FOR_MATLAB: {
            // Enable data logging for MATLAB
            sendLogToMatlab = true;
        } break;
        case COM_PRINT_HELP: {
            // Display help information
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = false;
        } break;
        case COM_GET_POSITION_LONG: {
            // Send current position as 32-bit value
            auto tempPos = Motor.getPosition( _Motor );
            writeBytes( tempPos, sizeof( tempPos ) );
        } break;
        case COM_LOG_POSITION: {
            // Set logging mode to position
            logWhat = LOG_POSITION;
        } break;
        case COM_LOG_CURRENT: {
            // Set logging mode to current
            logWhat = LOG_CURRENT;
        } break;
        case COM_GET_LOG_SIZE_INTEGER: {
            // Send log buffer size
            writeBytes( logSize, sizeof( logSize ) );
        } break;
        case COM_GET_LOOPRATE_UINT8: {
            // Send current loop rate
            Serial.write( velPID_1.getLoopRate() );
        } break;
        case COM_CHANGE_USER_POS_DIR: {
            // Toggle position direction and motor direction
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = false;
            Controller.disablePosPID( _Motor );
            Controller.disableVelPID( _Motor );
            uManual = 0;
            Motor.resetPosition( _Motor );
            if ( Motor.getUserPosDir( _Motor ) )
                Motor.setUserPosDir( false, _Motor );
            else
                Motor.setUserPosDir( true, _Motor );
            if ( Motor.getDir( _Motor ) )
                Motor.setDir( false, _Motor );
            else
                Motor.setDir( true, _Motor );
        } break;
        case COM_RESET_ENCODER_POS: {
            // Reset encoder position to zero
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = false;
            Controller.disablePosPID( _Motor );
            Controller.disableVelPID( _Motor );
            Controller.set_uManuel( 0, _Motor );
            Motor.resetPosition( _Motor );
        } break;
        case COM_GET_POS_GAINS: {
            // Send position PID gains
            _float2bits.Float = Controller.getPositionKp( _Motor );
            writeBytes( _float2bits.Hex, sizeof( uint32_t ) );

            _float2bits.Float = Controller.getPositionKd( _Motor );
            writeBytes( _float2bits.Hex, sizeof( uint32_t ) );

            _float2bits.Float = Controller.getPositionKi( _Motor );
            writeBytes( _float2bits.Hex, sizeof( uint32_t ) );
        } break;
        case COM_GET_VEL_GAINS: {
            // Send velocity PID gains
            _float2bits.Float = Controller.getVeloticyKp( _Motor );
            writeBytes( _float2bits.Hex, sizeof( uint32_t ) );
            _float2bits.Float = Controller.getVeloticyKi( _Motor );
            writeBytes( _float2bits.Hex, sizeof( uint32_t ) );
            _float2bits.Float = Controller.getVeloticyKd( _Motor );
            writeBytes( _float2bits.Hex, sizeof( uint32_t ) );
        } break;
    }
}

/**
 * @brief Process commands with one parameter
 * @param _Motor Motor index to operate on
 * @details Handles driver enable/disable, operating mode changes,
 *          reference value settings, and motor testing commands
 */
void SerialSolver::proccessSub2Cmd( uint8_t _Motor ) {
    switch ( _cmd.command ) {
        case COM_SET_DRIVER_STATE: {
            switch ( ( uint8_t ) _cmd.params[ 0 ] ) {
                case DISABLE:  // disable driver
                    Driver.disable();
                    Controller.zeroOutput();
                    Planner.disable();
                    digitalWrite( STATUS_LED_RED, LOW );
                    digitalWrite( STATUS_LED_BLUE, LOW );
                    break;
                case ENABLE:  // enable driver
                    Driver.enable();
                    Controller.zeroOutput();
                    Planner.disable();
                    digitalWrite( STATUS_LED_RED, HIGH );
                    break;
            }
        } break;
        case COM_SET_OPERATING_MODE: {
            Controller.set_uManuel( 0, _Motor );
            switch ( ( uint8_t ) _cmd.params[ 0 ] ) {
                case MANUAL_MODE:
                    Driver.setOperatingMode( MANUAL_MODE );
                    Controller.disablePosPID( _Motor );
                    Controller.disableVelPID( _Motor );
                    break;
                case POSITION_MODE:
                    Driver.setOperatingMode( POSITION_MODE );
                    Controller.setPositionReference( Motor.getPosition( _Motor ), _Motor );
                    Controller.enablePosPID( _Motor );
                    Controller.disableVelPID( _Motor );
                    digitalWrite( STATUS_LED_BLUE, HIGH );
                    break;
                case PROFILE_POSITION_MODE:
                    Planner.disable( _Motor );
                    Driver.setOperatingMode( PROFILE_POSITION_MODE );
                    posRef = Motor.getPosition( _Motor );
                    Controller.enablePosPID( _Motor );
                    Controller.disableVelPID( _Motor );
                    digitalWrite( STATUS_LED_BLUE, HIGH );
                    break;
                case VELOCITY_MODE:
                    Driver.setOperatingMode( VELOCITY_MODE );
                    Controller.disablePosPID( _Motor );
                    velRef = currentVel;
                    Controller.enableVelPID( _Motor );
                    break;
                case PROFILE_VELOCITY_MODE:
                    Driver.setOperatingMode( PROFILE_VELOCITY_MODE );
                    // do nothing
                    break;
                case CURRENT_MODE:
                    Driver.setOperatingMode( CURRENT_MODE );
                    // do nothing
                    break;
                case HOMING_MODE:
                    Driver.setOperatingMode( HOMING_MODE );
                    // do nothing
                    break;
            }
        } break;
        case COM_REL_MAX_PROF_POS_REF: {
            Planner.setRelativePositionReference( ( int32_t ) _cmd.params[ 0 ], _Motor );
            Planner.setInitialPosition( Motor.getPosition( _Motor ), _Motor );
            Planner.setFinalPosition(
                ( Planner.getInitialPosition( _Motor ) + Planner.getRelativePositionReference( _Motor ) ), _Motor );
            Planner.generatePosProfile( _Motor );
            // 6000 darbe öteye 0.7 saniye tatlı geçiş, o yüzden ref
            Planner.setTimeInSec( sqrt( abs( Planner.getRelativePositionReference( _Motor ) ) / 6000.0 * 0.7 ),
                                  _Motor );
            // 6000 darbe 0.7 saniye için max hız 16070
            if ( ( ( abs( Planner.getRelativePositionReference( _Motor ) ) / 6000.0 )
                   / ( Planner.getTimeInSec( _Motor ) / 0.7 ) * 16070.0 )
                 > MAX_PROF_VEL ) {
                Planner.setTimeInSec(
                    ( abs( Planner.getRelativePositionReference( _Motor ) ) / 6000.0 ) / ( MAX_PROF_VEL / 0.7 ) * 16070,
                    _Motor );
            }

            Planner.generatePosProfile( _Motor );

            float maxComputedAccel = Planner.evalAccel( Planner.getTimeInSec( _Motor ) / 4.73, _Motor );

            if ( abs( maxComputedAccel ) < MAX_PROF_ACCEL )
                Planner.enable( _Motor );
            else {
                Serial.print( F( "<" ) );
                Serial.print( rs485id );
                Serial.print( F( " ?>" ) );
            }
            Planner.setInitialTime( millis(), _Motor );
        } break;
        case COM_ABS_MAX_PROF_POS_REF: {
            Planner.setFinalPosition( ( int32_t ) _cmd.params[ 0 ], _Motor );
            Planner.setInitialPosition( Motor.getPosition( _Motor ), _Motor );
            Planner.setRelativePositionReference(
                ( Planner.getFinalPosition( _Motor ) - Planner.getInitialPosition( _Motor ) ), _Motor );
            Planner.setTimeInSec( sqrt( abs( Planner.getRelativePositionReference( _Motor ) ) / 6000.0 * 0.7 ),
                                  _Motor );

            if ( ( ( abs( Planner.getRelativePositionReference( _Motor ) ) / 6000.0 )
                   / ( Planner.getTimeInSec( _Motor ) / 0.7 ) * 16070.0 )
                 > MAX_PROF_VEL ) {
                Planner.setTimeInSec( ( ( abs( Planner.getRelativePositionReference( _Motor ) ) / 6000.0 )
                                        / ( MAX_PROF_VEL / 0.7 ) * 16070 ),
                                      _Motor );
            }

            Planner.generatePosProfile( _Motor );

            float maxComputedAccel = Planner.evalAccel( Planner.getTimeInSec( _Motor ) / 4.73, _Motor );

            if ( abs( maxComputedAccel ) < MAX_PROF_ACCEL )
                Planner.enable( _Motor );
            else
                Serial.println( F( "?" ) );

            Planner.setInitialTime( millis(), _Motor );
        } break;
        case COM_REL_VEL_REF: {
            velRef += _cmd.params[ 0 ];
        } break;
        case COM_ABS_VEL_REF: {
            velRef = _cmd.params[ 0 ];
        } break;
        case COM_REL_POS_REF: {
            posRef += ( int32_t ) _cmd.params[ 0 ];
        } break;
        case COM_ABS_POS_REF: {
            posRef = ( int32_t ) _cmd.params[ 0 ];
        } break;
        case COM_SET_U: {
            Controller.set_uManuel( constrain( ( int16_t ) _cmd.params[ 0 ], -255, 255 ), _Motor );
            logEnable = true;
        } break;
        case COM_TEST_MOTOR_CONNECTION: {
            longReportPos  = false;
            longReportVel  = false;
            shortReportPos = false;
            shortReportVel = false;
            Controller.disablePosPID( _Motor );
            Controller.disableVelPID( _Motor );
            Controller.set_uManuel( 0, _Motor );

            uint8_t uTemp = constrain( ( int16_t ) _cmd.params[ 0 ], 0, 255 );

            int32_t tempPos = Motor.getPosition( _Motor );

            Motor.setSpeed( uTemp, _Motor );
            delay( 100 );
            Motor.setSpeed( 0, _Motor );
            if ( ( Motor.getPosition( _Motor ) - tempPos ) > 0 )
                Serial.println( F( "passed" ) );
            else {
                if ( Motor.getDir( _Motor ) )
                    Motor.setDir( false, _Motor );
                else
                    Motor.setDir( true, _Motor );
                Serial.println( F( "First failed but corrected automatically" ) );
            }
            delay( 100 );
            Motor.setSpeed( uTemp, _Motor );
            delay( 100 );
            Motor.setSpeed( 0, _Motor );
        } break;
        case COM_SET_LOOPRATE: {
            uint8_t loopRateTemp = ( uint8_t ) _cmd.params[ 0 ];
            if ( loopRateTemp > 0 ) {
                posPID_1.setLoopRate( loopRateTemp );
                velPID_1.setLoopRate( loopRateTemp );
            }
        } break;
        case COM_SET_PRINTING_PERIOD: {
            uint16_t printingPeriodTemp = ( uint16_t ) _cmd.params[ 0 ];
            if ( printingPeriodTemp > 0 ) printingPeriod = printingPeriodTemp;
        } break;
    }
}

/**
 * @brief Process commands with two parameters
 * @param _Motor Motor index to operate on
 * @details Handles profile position commands with time parameters
 */
void SerialSolver::proccessSub3Cmd( uint8_t _Motor ) {
    switch ( _cmd.command ) {
        case COM_REL_PROF_POS_REF: {
            Planner.setRelativePositionReference( ( int32_t ) _cmd.params[ 0 ], _Motor );
            Planner.setTimeInSec( _cmd.params[ 1 ], _Motor );

            if ( Planner.getTimeInSec( _Motor ) > 0 ) {
                Planner.setInitialPosition( Motor.getPosition( _Motor ), _Motor );
                Planner.setFinalPosition(
                    ( Planner.getInitialPosition( _Motor ) + Planner.getRelativePositionReference( _Motor ) ), _Motor );
                Planner.generatePosProfile( _Motor );

                float maxComputedAccel = Planner.evalAccel( Planner.getTimeInSec( _Motor ) / 4.73, _Motor );

                if ( abs( maxComputedAccel ) < MAX_PROF_ACCEL )
                    Planner.enable( _Motor );
                else
                    Serial.println( F( "?" ) );

                Planner.setInitialTime( millis(), _Motor );
            }
        } break;
        case COM_ABS_PROF_POS_REF: {
            Planner.setFinalPosition( ( int32_t ) _cmd.params[ 0 ], _Motor );
            Planner.setTimeInSec( _cmd.params[ 1 ], _Motor );

            if ( Planner.getTimeInSec( _Motor ) > 0 ) {
                Planner.setInitialPosition( Motor.getPosition( _Motor ), _Motor );

                Planner.generatePosProfile( _Motor );

                float maxComputedAccel = Planner.evalAccel( Planner.getTimeInSec( _Motor ) / 4.73, _Motor );

                if ( abs( maxComputedAccel ) < MAX_PROF_ACCEL )
                    Planner.enable( _Motor );
                else
                    Serial.println( F( "?" ) );

                Planner.setInitialTime( millis(), _Motor );
            }
        } break;
    }
}

/**
 * @brief Process commands with three parameters  
 * @param _Motor Motor index to operate on
 * @details Handles PID gain setting commands (Kp, Ki, Kd)
 */
void SerialSolver::proccessSub4Cmd( uint8_t _Motor ) {
    switch ( _cmd.command ) {
        case COM_SET_POS_GAINS: {
            Controller.setPositionPIDGains( _Motor, _cmd.params[ 0 ], _cmd.params[ 1 ], _cmd.params[ 2 ] );
        } break;
        case COM_SET_VEL_GAINS: {
            Controller.setVelocityPIDGains( _Motor, _cmd.params[ 0 ], _cmd.params[ 1 ], _cmd.params[ 2 ] );
        } break;
    }
}