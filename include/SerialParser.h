#pragma once

#include <Arduino.h>

#include "conf.h"

#define MAX_DATA_NUM 5

class SerialParser {
   public:
    SerialParser() = default;
    void readCommands();

    bool    _Status               = 0;
    float   _Data[ MAX_DATA_NUM ] = { 0 };
    uint8_t _Index                = 0;
};