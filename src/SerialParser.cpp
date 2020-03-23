#include "SerialParser.h"

void SerialParser::readCommands() {
    static int8_t _correctionFlag = -1;
    static int    dummy;

    while ( Serial.available() > 0 )  // Seri port bufferında bilgi varmı
    {
        if ( _correctionFlag == -1 ) delayMicroseconds( 1000 );
        _correctionFlag = 1;
        if ( _Index < 5 ) {
            _Data[ _Index++ ] = Serial.parseFloat();
            dummy             = Serial.read();

            if ( dummy == '\r' || dummy == '\n' ) {
                _correctionFlag = 2;
                if ( Serial.available() > 0 ) {
                    Serial.read();
                }
            }
        } else {
            _correctionFlag = 2;
            Serial.parseFloat();
        }
    }

    if ( _correctionFlag == ( int8_t ) 1 ) {
        _Data[ --_Index ] = -1;
        _correctionFlag   = -1;
        _Status           = 1;
    } else if ( _correctionFlag == ( int8_t ) 2 ) {
        _correctionFlag = -1;
        _Status         = 1;
    }
}
