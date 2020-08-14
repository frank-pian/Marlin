#include "../MarlinCore.h"
#include "CNCManager.h"


void CNCManager::Init() {
    can_set_headpwr_en(0x01);
}


void CNCManager::SetPower(uint8_t Percent) {
    ChangePower(Percent);
    can_set_pwm(Percent);
}


void CNCManager::On() {
    SetPower(last_percent);
}


void CNCManager::Off() {
    can_set_pwm(0);
}


void CNCManager::ChangePower(uint8_t Percent) {
    last_percent = Percent;
}