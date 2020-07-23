#pragma once

class Manager
{
  public:
    void Init();
    uint8_t HeadType;
    void GetHeadType();
};

extern Manager HeadManager;