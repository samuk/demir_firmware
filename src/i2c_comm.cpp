/**
 * @file i2c_comm.cpp
 * @brief I2C communication interface implementation for DEMIR controller
 * @author Muhammet Şam Rossiter
 * @date 2025
 */

#include "i2c_comm.h"

uint8_t i2cCmdBuffer[I2C_CMD_BUF_SIZE];
uint8_t i2cCmdIndex = 0;

void receiveI2CCommand(int byteCount) {
    while (Wire.available() && i2cCmdIndex < I2C_CMD_BUF_SIZE) {
        i2cCmdBuffer[i2cCmdIndex++] = Wire.read();
    }
    if (i2cCmdIndex > 0) {
        Solver.proccessI2CCommand(i2cCmdBuffer, i2cCmdIndex);
        i2cCmdIndex = 0;
    }
}

void sendI2CResponse() {
    uint32_t pos = Motor.getPosition(MOTOR_1);
    Wire.write((uint8_t*)&pos, sizeof(pos));
}
