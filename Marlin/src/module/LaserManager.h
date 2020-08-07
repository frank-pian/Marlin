#pragma once

#include "../inc/MarlinConfig.h"

#define LASER_POWER_MAX_LIMIT (100)

class LaserManager
{
    public:
        LaserManager(){};
        void Init();
        void Off();
        void On();
        void SetPower(uint8_t Percent);
        void SetPwm(uint8_t pwm_value);
        void ChangePowerLimit(uint8_t limit);
        void ChangePower(uint8_t percent);
        uint8_t GetPwm();
        uint8_t GetPowerPercent() { return last_percent; }

    private:
        uint8_t  last_pwm;
        uint8_t    power_limit_;
        uint8_t    last_percent;
};