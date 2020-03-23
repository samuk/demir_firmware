#include "MotorDriver.h"
MotorDriver Driver;

void MotorDriver::init() {
    Motor.init();
    Controller.init();
}

void MotorDriver::run() {
    Controller.updateCurrentPosition();
    Controller.updateCurrentVelocity();

    if ( state() ) {
        switch ( getOperatingMode() ) {
            case MANUAL_MODE:  // elle sürüş
                Controller.manuelMode();
                break;
            case POSITION_MODE:  // pozisyon kontrol
                Controller.positionMode();
                break;
            case PROFILE_POSITION_MODE:  // profilli pozisyon kontrol modu
                Controller.profilePositionMode();
                break;
            case VELOCITY_MODE:
                Controller.velocityMode();
                break;
            case PROFILE_VELOCITY_MODE:
                // Do nothing
                break;
            case CURRENT_MODE:
                // Do nothing
                break;
            case HOMING_MODE:
                // Do nothing
                break;
        }
    } else {
        Controller.zeroOutput();
    }
    Controller.update();
}