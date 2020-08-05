#include "../inc/MarlinConfig.h"

#include "../MarlinCore.h"
#include "LaserManager.h"

static const uint16_t LaserPowerTable[]=
{
  0,
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};

void LaserManager::Init()
{
    can_set_headpwr_en(0x01);
    power_limit_ = LASER_POWER_MAX_LIMIT;
}

void LaserManager::SetPwm(const uint8_t pwm_value)
{
    can_set_pwm(pwm_value);
}

void LaserManager::SetPower(float Percent)
{
    ChangePower(Percent);
    SetPwm(last_pwm);
}

/**
 * Off:Laser off without changing the power
 */
void LaserManager::Off() {

  SetPwm(0);
}

/**
 * On:Laser on and use the last power
 */
void LaserManager::On() {

  SetPower(last_percent);
}

/**
 * change power limit, will be call when open / close chamber door
*/
void LaserManager::ChangePowerLimit(float limit) {
  float percent = last_percent;
  if (limit > LASER_POWER_MAX_LIMIT)
    limit = LASER_POWER_MAX_LIMIT;
  power_limit_ = limit;
  // if previous limit is larger than now, need to check need to lower current output
  ChangePower(last_percent);
  last_percent = percent;  // recover the value of the normal output
  
//   if (GetTimPwm() > 0) {
//     // If there is current output, it is equal to the limit output
//     setPwm(last_pwm);
//   }
}

/**
 * change power value, but not change output power
*/
void LaserManager::ChangePower(float percent)
{
    int integer;
    float decimal;
    last_percent = percent;
    if (percent > power_limit_)
        percent = power_limit_;

    //integer = percent;
    //decimal = percent - integer;
    //last_pwm = LaserPowerTable[integer] + (LaserPowerTable[integer + 1] - LaserPowerTable[integer]) * decimal;
    last_pwm = percent;
}