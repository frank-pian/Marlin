#pragma once

#include "LaserManager.h"
#include "CNCManager.h"

class Manager
{
  public:
    void Init();
    uint8_t HeadType;
    void GetHeadType();

    LaserManager Laser;
    CNCManager CNC;
};

extern Manager HeadManager;