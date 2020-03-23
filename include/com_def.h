#pragma once
#include "MotorDriver.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "conf.h"
#include "motor.h"

#define COM_REL_VEL_REF           1
#define COM_ABS_VEL_REF           2
#define COM_REL_POS_REF           3
#define COM_ABS_POS_REF           4
#define COM_SET_POS_GAINS         5
#define COM_SET_VEL_GAINS         6
#define COM_SET_U                 13
#define COM_TEST_MOTOR_CONNECTION 18

#define COM_PRINT_LONG_VEL_REPORT  7
#define COM_PRINT_SHORT_VEL_REPORT 8
#define COM_PRINT_LONG_POS_REPORT  9
#define COM_PRINT_SHORT_POS_REPORT 10
#define COM_PRINT_NO_REPORT        11
#define COM_SEND_DATA_FOR_MATLAB   12
#define COM_PRINT_HELP             14
#define COM_GET_POSITION_LONG      15
#define COM_LOG_POSITION           16
#define COM_LOG_CURRENT            17
#define COM_GET_LOG_SIZE_INTEGER   19
#define COM_GET_LOOPRATE_UINT8     20
#define COM_CHANGE_USER_POS_DIR    21
#define COM_RESET_ENCODER_POS      24
#define COM_GET_POS_GAINS          25
#define COM_GET_VEL_GAINS          26

#define COM_SET_LOOPRATE         22
#define COM_SET_PRINTING_PERIOD  23
#define COM_REL_PROF_POS_REF     27
#define COM_ABS_PROF_POS_REF     28
#define COM_SET_DRIVER_STATE     29
#define COM_SET_OPERATING_MODE   30
#define COM_REL_MAX_PROF_POS_REF 31
#define COM_ABS_MAX_PROF_POS_REF 32
//
//

//
//
//#define MANUAL_MODE           0
//#define POSITION_MODE         1
//#define PROFILE_POSITION_MODE 2
//#define VELOCITY_MODE         3
//#define PROFILE_VELOCITY_MODE 4
//#define CURRENT_MODE          5
//#define HOMING_MODE           6

void COM_command_1();
void COM_command_2();
void COM_command_3();
void COM_command_4();
void COM_command_5();
void COM_command_6();
void COM_command_7();
void COM_command_8();
void COM_command_9();
void COM_command_10();
void COM_command_11();
void COM_command_12();
void COM_command_13();
void COM_command_14();
void COM_command_15();
void COM_command_16();
void COM_command_17();
void COM_command_17();
void COM_command_18();
void COM_command_19();
void COM_command_20();
void COM_command_21();
void COM_command_22();
void COM_command_23();
void COM_command_24();
void COM_command_25();
void COM_command_26();
void COM_command_27();
void COM_command_28();
void COM_command_29( uint8_t state );
void COM_command_30();
void COM_command_31();
void COM_command_32();
