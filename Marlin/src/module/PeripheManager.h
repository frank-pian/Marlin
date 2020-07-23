#pragma once

#include "../inc/MarlinConfig.h"

class PeripheManager
{
    public:
        bool door_check_auto_switch = false;
        void Init();
        void DoorCheck();
        void DoorCheckAuto();
        void SetDoorCheckStatus(bool Enable);
    private:
        bool last_door_status;
};

extern PeripheManager Periphe;