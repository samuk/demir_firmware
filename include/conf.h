#pragma once
//#include "com_def.h"

#define FORCE_INLINE __attribute__( ( always_inline ) ) inline

#define TIMER1_A_PIN 9
#define TIMER1_B_PIN 10

#define VNH7070
#define BAUDRATE 500000
#define TIMEOUT  100

#define ANALOG_WRITE_RES 9
//#define NUM_OF_MOTOR     1
#define MOTORS 1
#define PWMS   6

#define MAX_PROF_ACCEL 19000.0  // Enc pulse per second square
#define MAX_PROF_VEL   19000.0  // Enc pulse per second

#define LOG_SIZE_MAX 200

#define LOG_POSITION 0  // Pozisyon kaydı tutmak
#define LOG_CURRENT  1  // Akım kaydı tutmak

#define DISABLED 0  // Sürücü pasif
#define ENABLED  1  // Sürücü aktif

#define DISABLE 0
#define ENABLE  1

#define STATUS_LED_BLUE 5
#define STATUS_LED_RED  6

#define MOTOR1_CURRENT_SENSE A1
#define MOTOR1_INA           8
#define MOTOR1_INB           7
#define MOTOR1_PWM           9
#define MOTOR1_SEL0          4
#define MOTOR1_encPinA       2
#define MOTOR1_encPinB       3

#define MOTOR2_CURRENT_SENSE
#define MOTOR2_INA
#define MOTOR2_INB
#define MOTOR2_PWM
#define MOTOR2_SEL0
#define MOTOR2_encPinA
#define MOTOR2_encPinB

#define MOTOR3_CURRENT_SENSE
#define MOTOR3_INA
#define MOTOR3_INB
#define MOTOR3_PWM
#define MOTOR3_SEL0
#define MOTOR3_encPinA

#define MOTOR4_CURRENT_SENSE
#define MOTOR4_INA
#define MOTOR4_INB
#define MOTOR4_PWM
#define MOTOR4_SEL0
#define MOTOR4_encPinA
#define MOTOR4_encPinB