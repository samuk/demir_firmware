/**
 * @file controller.h
 * @brief Main control system coordination class declaration
 * @author Muhammet Şükrü Demir
 * @date 2020
 * @details Header file for controller class which coordinates PID controllers,
 *          sensor feedback, and different control modes for precise motor control.
 */

#pragma once

#include <Arduino.h>

#include "Motion.h"
#include "MotorDriver.h"
#include "PID.h"
#include "PLANNER.h"
#include "SerialParser.h"
#include "board/DEMIRv1/pins_DEMIRv1.h"
#include "com_def.h"
#include "conf.h"
#include "motor.h"

/**
 * @class controller
 * @brief Main control system coordination class
 * @details Coordinates PID controllers, sensor feedback processing, and execution
 *          of different control modes. Handles position and velocity control loops,
 *          trajectory execution, and sensor data filtering for precise motor control.
 */
class controller {
   private:
    float   u[ MOTORS ]                   = { 0 };     ///< Control output signals
    float   uManuel[ MOTORS ]             = { 0 };     ///< Manual control signals
    int32_t positionReference[ MOTORS ]   = { 0 };     ///< Position reference values
    int32_t currentPosition[ MOTORS ]     = { 0 };     ///< Current position readings
    int32_t previousPosition[ MOTORS ]    = { 0 };     ///< Previous position for velocity calculation
    float   currentVelocity[ MOTORS ]     = { 0 };     ///< Filtered velocity estimates
    float   previousVelocity[ MOTORS ]    = { 0 };     ///< Previous velocity for filtering
    float   velocityFilterInput[ MOTORS ] = { 0 };     ///< Raw velocity calculations
    float   velocityReference[ MOTORS ]   = { 0 };     ///< Velocity reference values
    float   alpha                         = 0.2f;      ///< Velocity filter coefficient

   public:
    controller() = default;  ///< Default constructor

    /**
     * @brief Initialize control system
     * @details Sets up PID controllers with default gains
     */
    void init();
    
    /**
     * @brief Zero all motor control outputs
     */
    void zeroOutput();
    
    /**
     * @brief Set manual control output
     * @param uManuel Manual control value
     * @param _Motor Motor index
     */
    void set_uManuel( uint8_t uManuel, uint8_t _Motor );
    
    /**
     * @brief Execute manual control mode
     */
    void manuelMode();
    
    /**
     * @brief Execute position control mode
     */
    void positionMode();
    
    /**
     * @brief Execute profile position control mode
     */
    void profilePositionMode();
    
    /**
     * @brief Execute velocity control mode
     */
    void velocityMode();
    
    /**
     * @brief Execute profile velocity control mode
     */
    void profileVelocityMode();
    
    /**
     * @brief Update velocity estimates from position feedback
     */
    void updateCurrentVelocity();

    /**
     * @brief Set position reference for motor
     * @param ref Position reference
     * @param Motor Motor index
     */
    void setPositionReference( int32_t ref, uint8_t Motor );
    
    /**
     * @brief Update current position readings
     */
    void updateCurrentPosition();
    
    /**
     * @brief Apply control outputs to motors
     */
    void update();

    // PID controller management
    void enablePosPID( uint8_t Motor );    ///< Enable position PID
    void disablePosPID( uint8_t Motor );   ///< Disable position PID
    void enableVelPID( uint8_t Motor );    ///< Enable velocity PID
    void disableVelPID( uint8_t Motor );   ///< Disable velocity PID

    // PID gain getters
    float getPositionKp( uint8_t Motor );  ///< Get position PID Kp
    float getPositionKi( uint8_t Motor );  ///< Get position PID Ki
    float getPositionKd( uint8_t Motor );  ///< Get position PID Kd
    float getVeloticyKp( uint8_t Motor );  ///< Get velocity PID Kp
    float getVeloticyKi( uint8_t Motor );  ///< Get velocity PID Ki
    float getVeloticyKd( uint8_t Motor );  ///< Get velocity PID Kd

    // PID gain setters
    void setPositionPIDGains( uint8_t Motor, float Kp, float Ki, float Kd );  ///< Set position PID gains
    void setVelocityPIDGains( uint8_t Motor, float Kp, float Ki, float Kd );  ///< Set velocity PID gains

    unsigned long deltaT      = 0;  ///< Time since last control update
    unsigned long currentTime = 0;  ///< Current timestamp
    unsigned long oldTime     = 0;  ///< Previous control loop timestamp
};

/// @brief Global controller instance
extern controller Controller;