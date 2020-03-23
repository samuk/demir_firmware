#include <Arduino.h>

#include "Motion.h"
#include "MotorDriver.h"
#include "SerialSolver.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "com_def.h"
#include "conf.h"
#include "controller.h"

uint8_t loopRate = 3;

void setup() {
    Serial.begin( BAUDRATE );
    while ( !Serial ) {
    }
    Serial.setTimeout( TIMEOUT );

    Serial.println( F( "DEMIR v1.1.1" ) );
    Driver.init();
    Serial.println( F( "ok!" ) );

    // LED Test
    pinMode( STATUS_LED_BLUE, OUTPUT );
    pinMode( STATUS_LED_RED, OUTPUT );

    digitalWrite( STATUS_LED_BLUE, HIGH );
    delay( 300 );
    digitalWrite( STATUS_LED_BLUE, LOW );

    digitalWrite( STATUS_LED_RED, HIGH );
    delay( 300 );
    digitalWrite( STATUS_LED_RED, LOW );
}
void loop() {
    Controller.currentTime = millis();  //Şimdiki zaman
    Controller.deltaT
        = Controller.currentTime - Controller.oldTime;  //Şimdiki zaman - Bir önceki PID çalıştırılma zamanı
    if ( Controller.deltaT >= loopRate )  // deltaT loopRate'den büyük veya eşitse PID çalıştırılır
    {
        Controller.oldTime = Controller.currentTime;
        Driver.run();

        if ( Solver.logEnable )  // Eğer kayıt yetkilendirilmişse
        {
            switch ( Solver.logWhat )  // neyin kaydedileceğine bakılıyor
            {
                case LOG_POSITION:
                    Solver.logForMatlab[ Solver.logCounter ] = ( unsigned long ) Motor.getPosition( MOTOR_1 );
                    break;
                case LOG_CURRENT:
                    // unsigned long tempHex             = *( unsigned long* ) &totalCurrent;
                    // logForMatlab[ Solver.logCounter ] = tempHex;
                    break;
            }
            if ( ++Solver.logCounter == Solver.logSize ) {
                Solver.logEnable  = false;
                Solver.logCounter = 0;
            }
        }
    }

    Solver.proccessCommands();

    if ( Solver.sendLogToMatlab ) {
        Solver.sendLogToMatlab = false;

        for ( uint16_t j = 0; j < Solver.logSize; j++ ) {
            Serial.write( Solver.logForMatlab[ j ] );  // dört bytelık bilgi sırayla kaydırılarak gönderiliyor
            Serial.write( Solver.logForMatlab[ j ] >> 8 );
            Serial.write( Solver.logForMatlab[ j ] >> 16 );
            Serial.write( Solver.logForMatlab[ j ] >> 24 );
        }
    }
}
