#include "PID.h"

PID posPID_1;
PID velPID_1;

#if ( MOTORS > 1 )
PID posPID_2;
PID velPID_2;
#endif

#if ( MOTORS > 2 )
PID posPID_3;
PID velPID_3;
#endif

#if ( MOTORS > 3 )
PID posPID_4;
PID velPID_4;
#endif

PID* velPID[ MOTORS ] = { &velPID_1 };
PID* posPID[ MOTORS ] = { &posPID_1 };

PID::PID()
    : _enabled( false ),
      _u( 0.0f ),
      _kP( 0.0f ),
      _kI( 0.0f ),
      _kD( 0.0f ),
      _error( 0.0f ),
      _previousError( 0.0f ),
      _uP( 0.0f ),
      _uI( 0.0f ),
      _uD( 0.0f ),
      _outMax( 255.0f ),
      _outMin( -255.0f ),
      _currentTime( 0 ),
      _oldTime( 0 ),
      _deltaT( 0 ),
      _loopRate( 3 ) {}

void PID::enable() { _enabled = true; }

void PID::disable() {
    _enabled       = false;
    _u             = 0.0f;
    _uI            = 0.0f;
    _previousError = 0.0f;
}

uint8_t PID::compute( float err ) {
    if ( !getStatus() ) return -1;

    _currentTime = millis();
    _deltaT      = _currentTime - _oldTime;

    if ( _deltaT >= _loopRate ) {
        _oldTime = _currentTime;

        _error = err;

        // P
        _uP = _kP * _error;
        //

        // I
        _uI += _kI * _error * float( _deltaT ) / 1000.0f;
        _uI = constrain( _uI, _outMin, _outMax );
        //

        // D
        _uD            = _kD * ( _error - _previousError ) * 1000.0f / float( _deltaT );
        _previousError = _error;
        //

        _u = _uP + _uI + _uD;
        _u = constrain( _u, _outMin, _outMax );

        return 1;
    }
    return 0;
}

float PID::compute( float err, float deltaT ) {
    if ( !getStatus() ) return 0;

    _error = err;

    // P
    _uP = _kP * _error;
    //

    // I
    _uI += _kI * _error * deltaT / 1000.0f;
    _uI = constrain( _uI, _outMin, _outMax );

    // D
    _uD            = _kD * ( _error - _previousError ) * 1000.0f / deltaT;
    _previousError = _error;
    //

    _u = _uP + _uI + _uD;
    _u = constrain( _u, _outMin, _outMax );

    return _u;
}

void PID::setKp( const float kP ) {
    if ( kP < 0 ) return;
    _kP = kP;
}

void PID::setKi( const float kI ) {
    if ( kI < 0 ) return;
    _kI = kI;
}

void PID::setKd( const float kD ) {
    if ( kD < 0 ) return;
    _kD = kD;
}

void PID::setGains( const float kP, const float kI, const float kD ) {
    if ( kP < 0 || kI < 0 || kD < 0 ) return;
    _kP = kP;
    _kI = kI;
    _kD = kD;
}

void PID::setOutMinMax( const float outMin, const float outMax ) {
    if ( outMin < outMax ) {
        _outMin = outMin;
        _outMax = outMax;
    }
}
