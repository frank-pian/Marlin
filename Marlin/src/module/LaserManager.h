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
        void SetPower(float Percent);
        void SetPwm(uint8_t pwm_value);
        void ChangePowerLimit(float limit);
        void ChangePower(float percent);

    private:
        uint8_t  last_pwm;
        float    power_limit_;
        float    last_percent;
};