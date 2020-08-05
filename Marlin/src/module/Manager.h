#pragma once

#include "LaserManager.h"

class Manager
{
  public:
    void Init();
    uint8_t HeadType;
    void GetHeadType();

    LaserManager Laser;
};

extern Manager HeadManager;