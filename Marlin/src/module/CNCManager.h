#pragma once

#include "../inc/MarlinConfig.h"

class CNCManager
{
    public:
        CNCManager(){};
        void Init();
        void SetPower(uint8_t Percent);
        void On();
        void Off();
        void ChangePower(uint8_t Percent);
        uint8_t GetPower() { return last_percent; };

    private:
        uint8_t last_percent;
};