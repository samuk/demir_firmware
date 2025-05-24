/**
 * @file com_def.h
 * @brief Communication protocol command definitions
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file containing command identifiers and function declarations
 *          for the DEMIR motor controller communication protocol.
 */

#pragma once
#include "MotorDriver.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "conf.h"
#include "motor.h"

// Motion control commands
#define COM_REL_VEL_REF           1   ///< Set relative velocity reference
#define COM_ABS_VEL_REF           2   ///< Set absolute velocity reference
#define COM_REL_POS_REF           3   ///< Set relative position reference
#define COM_ABS_POS_REF           4   ///< Set absolute position reference
#define COM_SET_POS_GAINS         5   ///< Set position PID gains
#define COM_SET_VEL_GAINS         6   ///< Set velocity PID gains
#define COM_SET_U                 13  ///< Set manual control output
#define COM_TEST_MOTOR_CONNECTION 18  ///< Test motor connection

// Reporting and monitoring commands
#define COM_PRINT_LONG_VEL_REPORT  7   ///< Enable detailed velocity reporting
#define COM_PRINT_SHORT_VEL_REPORT 8   ///< Enable short velocity reporting
#define COM_PRINT_LONG_POS_REPORT  9   ///< Enable detailed position reporting
#define COM_PRINT_SHORT_POS_REPORT 10  ///< Enable short position reporting
#define COM_PRINT_NO_REPORT        11  ///< Disable all reporting
#define COM_SEND_DATA_FOR_MATLAB   12  ///< Send logged data to MATLAB
#define COM_PRINT_HELP             14  ///< Print help information
#define COM_GET_POSITION_LONG      15  ///< Get current position (32-bit)
#define COM_LOG_POSITION           16  ///< Set logging mode to position
#define COM_LOG_CURRENT            17  ///< Set logging mode to current
#define COM_GET_LOG_SIZE_INTEGER   19  ///< Get log buffer size
#define COM_GET_LOOPRATE_UINT8     20  ///< Get current loop rate
#define COM_CHANGE_USER_POS_DIR    21  ///< Toggle user position direction
#define COM_RESET_ENCODER_POS      24  ///< Reset encoder position to zero
#define COM_GET_POS_GAINS          25  ///< Get position PID gains
#define COM_GET_VEL_GAINS          26  ///< Get velocity PID gains

// System configuration commands
#define COM_SET_LOOPRATE         22  ///< Set control loop rate
#define COM_SET_PRINTING_PERIOD  23  ///< Set reporting period
#define COM_REL_PROF_POS_REF     27  ///< Set relative profile position reference
#define COM_ABS_PROF_POS_REF     28  ///< Set absolute profile position reference
#define COM_SET_DRIVER_STATE     29  ///< Set driver enable/disable state
#define COM_SET_OPERATING_MODE   30  ///< Set motor operating mode
#define COM_REL_MAX_PROF_POS_REF 31  ///< Set relative profile position with max constraints
#define COM_ABS_MAX_PROF_POS_REF 32  ///< Set absolute profile position with max constraints

/**
 * @brief Command handler function declarations
 * @details Most command handlers are implemented as placeholder functions,
 *          with actual implementation in SerialSolver class methods.
 */
void COM_command_1();   ///< Command handler 1
void COM_command_2();   ///< Command handler 2
void COM_command_3();   ///< Command handler 3
void COM_command_4();   ///< Command handler 4
void COM_command_5();   ///< Command handler 5
void COM_command_6();   ///< Command handler 6
void COM_command_7();   ///< Command handler 7
void COM_command_8();   ///< Command handler 8
void COM_command_9();   ///< Command handler 9
void COM_command_10();  ///< Command handler 10
void COM_command_11();  ///< Command handler 11
void COM_command_12();  ///< Command handler 12
void COM_command_13();  ///< Command handler 13
void COM_command_14();  ///< Command handler 14
void COM_command_15();  ///< Command handler 15
void COM_command_16();  ///< Command handler 16
void COM_command_17();  ///< Command handler 17 (duplicate declaration)
void COM_command_17();  ///< Command handler 17
void COM_command_18();  ///< Command handler 18
void COM_command_19();  ///< Command handler 19
void COM_command_20();  ///< Command handler 20
void COM_command_21();  ///< Command handler 21
void COM_command_22();  ///< Command handler 22
void COM_command_23();  ///< Command handler 23
void COM_command_24();  ///< Command handler 24
void COM_command_25();  ///< Command handler 25
void COM_command_26();  ///< Command handler 26
void COM_command_27();  ///< Command handler 27
void COM_command_28();  ///< Command handler 28

/**
 * @brief Driver state control command handler
 * @param state Driver state (DRIVER_DISABLE or DRIVER_ENABLE)
 * @details Active command handler for controlling driver enable/disable state
 */
void COM_command_29( uint8_t state );

void COM_command_30();  ///< Command handler 30
void COM_command_31();  ///< Command handler 31
void COM_command_32();  ///< Command handler 32
