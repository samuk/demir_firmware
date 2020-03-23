#include "motor.h"

#include "board/DEMIRv1/pins_DEMIRv1.h"

motor Motor;
motor::motor() {}

void motor::init() {
    motor_1.init( MOTOR1_INA, MOTOR1_INB, MOTOR1_PWM, MOTOR1_SEL0, MOTOR1_CURRENT_SENSE );
    encoder_1.init( MOTOR1_encPinA, MOTOR1_encPinB );
#if MOTORS > 1
    motor_2.init( MOTOR2_INA, MOTOR2_INB, MOTOR2_PWM, MOTOR2_SEL0, MOTOR2_CURRENT_SENSE );
    encoder_2.init( MOTOR2_encPinA, MOTOR2_encPinB );
#endif
#if MOTORS > 2
    motor_2.init( MOTOR3_INA, MOTOR3_INB, MOTOR1_PWM, MOTOR3_SEL0, MOTOR3_CURRENT_SENSE );
    encoder_3.init( MOTOR3_encPinA, MOTOR3_encPinB );
#endif
#if MOTORS > 3
    motor_4.init( MOTOR4_INA, MOTOR4_INB, MOTOR4_PWM, MOTOR4_SEL0, MOTOR4_CURRENT_SENSE );
    encoder_4.init( MOTOR4_encPinA, MOTOR4_encPinB );
#endif
}

void motor::setUserPosDir( bool dir, uint8_t Motor ) { Encoders[ Motor ]->setUserPosDir( dir ); }

bool motor::getUserPosDir( uint8_t Motor ) { return Encoders[ Motor ]->getUserPosDir(); }

// resetPosition(ENCODER_1) like that :)
void motor::resetPosition( uint8_t Motor ) { Encoders[ Motor ]->resetPosition(); }
// setSpeed(speed,MOTOR_1) like that
void motor::setSpeed( uint8_t motor_speed, uint8_t Motor ) { Motors[ Motor ]->uToPWM( motor_speed ); }

bool motor::getDir( uint8_t Motor ) { return Motors[ Motor ]->getMotorDir(); }

void motor::setDir( bool dir, uint8_t Motor ) { Motors[ Motor ]->setMotorDir( dir ); }

int32_t motor::getPosition( uint8_t Motor ) { return Encoders[ Motor ]->getPosition(); }

void motor::enable( uint8_t Motor ) { Motors[ Motor ]->setStatus( ENABLE ); }

void motor::disable( uint8_t Motor ) { Motors[ Motor ]->setStatus( DISABLE ); }

bool motor::isEnabled( uint8_t Motor ) { return Motors[ Motor ]->getStatus(); }

bool motor::isDisabled( uint8_t Motor ) { return !Motors[ Motor ]->getStatus(); }
