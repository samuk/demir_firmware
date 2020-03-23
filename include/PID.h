#pragma once

#include <Arduino.h>

#include "conf.h"

class PID {
   public:
    PID();

    uint8_t compute( float err );
    float   compute( float err, float deltaT );

    void enable();
    void disable();
    void setKp( const float kP );
    void setKi( const float kI );
    void setKd( const float kD );
    void setGains( const float kP = 0, const float kI = 0, const float kD = 0 );
    void setOutMinMax( const float outMin, const float outMax );
    void setLoopRate( const uint8_t loopRate ) { this->_loopRate = loopRate; }

    float   getU() const { return _u; }
    float   getKp() const { return _kP; }
    float   getKi() const { return _kI; }
    float   getKd() const { return _kD; }
    float   getUp() const { return _uP; }
    float   getUi() const { return _uI; }
    float   getUd() const { return _uD; }
    bool    getStatus() const { return _enabled; }
    uint8_t getLoopRate() const { return _loopRate; }

   private:
    bool          _enabled;
    float         _u;
    float         _kP;
    float         _kI;
    float         _kD;
    float         _error;
    float         _previousError;
    float         _uP;
    float         _uI;
    float         _uD;
    float         _outMax;
    float         _outMin;
    unsigned long _currentTime;
    unsigned long _oldTime;
    unsigned long _deltaT;
    uint8_t       _loopRate;
};

extern PID posPID_1;
extern PID velPID_1;

#if ( MOTORS >= 2 )
extern PID posPID_2;
extern PID velPID_2;
#endif

#if ( MOTORS >= 3 )
extern PID posPID_3;
extern PID velPID_3;
#endif

#if ( MOTORS >= 4 )
extern PID posPID_4;
extern PID velPID_4;
#endif

extern PID* velPID[ MOTORS ];
extern PID* posPID[ MOTORS ];