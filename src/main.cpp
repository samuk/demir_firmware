/**
 * @file main.cpp
 * @brief Main application entry point for DEMIR motor controller
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @version 1.1.1
 * @details This is the main application file that initializes the DEMIR motor control
 *          system and implements the main control loop with serial command processing,
 *          motor control execution, and data logging capabilities.
 */

#include <Arduino.h>

#include "Motion.h"
#include "MotorDriver.h"
#include "SerialSolver.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "com_def.h"
#include "conf.h"
#include "controller.h"

/// @brief Main control loop rate in milliseconds
uint8_t loopRate = 3;

/**
 * @brief Arduino setup function - system initialization
 * @details Initializes serial communication, motor drivers, and status LEDs.
 *          Performs LED test sequence to verify hardware functionality.
 */
void setup() {
    // Initialize serial communication
    Serial.begin( BAUDRATE );
    while ( !Serial ) {
        // Wait for serial port to initialize
    }
    Serial.setTimeout( TIMEOUT );

    // Print system identification
    Serial.println( F( "DEMIR v1.1.1" ) );
    
    // Initialize motor driver subsystem
    Driver.init();
    Serial.println( F( "ok!" ) );

    // Initialize status LEDs
    pinMode( STATUS_LED_BLUE, OUTPUT );
    pinMode( STATUS_LED_RED, OUTPUT );

    // LED startup test sequence
    digitalWrite( STATUS_LED_BLUE, HIGH );
    delay( 300 );
    digitalWrite( STATUS_LED_BLUE, LOW );

    digitalWrite( STATUS_LED_RED, HIGH );
    delay( 300 );
    digitalWrite( STATUS_LED_RED, LOW );
}

/**
 * @brief Arduino main loop function
 * @details Implements the main control loop with timing control, motor driver execution,
 *          data logging, and serial command processing. Runs continuously at specified
 *          loop rate for real-time motor control.
 */
void loop() {
    // Update timing for control loop
    Controller.currentTime = millis();  ///< Current timestamp
    Controller.deltaT
        = Controller.currentTime - Controller.oldTime;  ///< Time since last control update

    // Execute control loop at specified rate
    if ( Controller.deltaT >= loopRate )  // Execute when deltaT exceeds loopRate
    {
        Controller.oldTime = Controller.currentTime;
        
        // Execute motor control algorithms
        Driver.run();

        // Handle data logging if enabled
        if ( Solver.logEnable )  // Check if logging is enabled
        {
            switch ( Solver.logWhat )  // Determine what to log
            {
                case LOG_POSITION:
                    /// Log current motor position
                    Solver.logForMatlab[ Solver.logCounter ] = ( unsigned long ) Motor.getPosition( MOTOR_1 );
                    break;
                case LOG_CURRENT:
                    /// Log current sensor data (not implemented)
                    // unsigned long tempHex             = *( unsigned long* ) &totalCurrent;
                    // logForMatlab[ Solver.logCounter ] = tempHex;
                    break;
            }
            
            // Check if log buffer is full
            if ( ++Solver.logCounter == Solver.logSize ) {
                Solver.logEnable  = false;  // Disable logging
                Solver.logCounter = 0;      // Reset counter
            }
        }
    }

    // Process incoming serial commands
    Solver.proccessCommands();

    // Send logged data to MATLAB if requested
    if ( Solver.sendLogToMatlab ) {
        Solver.sendLogToMatlab = false;

        // Transmit logged data as 4-byte values
        for ( uint16_t j = 0; j < Solver.logSize; j++ ) {
            Serial.write( Solver.logForMatlab[ j ] );        ///< LSB
            Serial.write( Solver.logForMatlab[ j ] >> 8 );   ///< Byte 1
            Serial.write( Solver.logForMatlab[ j ] >> 16 );  ///< Byte 2
            Serial.write( Solver.logForMatlab[ j ] >> 24 );  ///< MSB
        }
    }
}
