#pragma once

#include "Motion.h"
#include "board/DEMIRv1/hardware.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "conf.h"
#include "controller.h"
#include "motor.h"

typedef enum : uint8_t {
    MANUAL_MODE = 0,
    POSITION_MODE,
    PROFILE_POSITION_MODE,
    VELOCITY_MODE,
    PROFILE_VELOCITY_MODE,
    CURRENT_MODE,
    HOMING_MODE
} operatingMode;

typedef enum : uint8_t { DRIVER_DISABLE = 0, DRIVER_ENABLE } driverState;

class MotorDriver {
   public:
    MotorDriver() = default;
    void    init();
    uint8_t state() { return _state; }
    void    run();
    void    enable() {
        _state  = DRIVER_ENABLE;
        _opMode = MANUAL_MODE;
    }
    void disable() {
        _state  = DRIVER_DISABLE;
        _opMode = MANUAL_MODE;
    }
    uint8_t isEnabled() { return _state; }
    uint8_t isDisabled() { return !_state; }

    void    setOperatingMode( uint8_t mode ) { _opMode = mode; }
    uint8_t getOperatingMode() { return _opMode; }

   private:
    uint8_t _state  = DRIVER_DISABLE;
    uint8_t _opMode = MANUAL_MODE;
};

extern MotorDriver Driver;