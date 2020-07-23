#include "../inc/MarlinConfig.h"
#include "Manager.h"
#include "../MarlinCore.h"

Manager HeadManager;

void Manager::Init()
{
    HeadType = HEAD_TYPE_UNDEFINE;
    GetHeadType();
}

void Manager::GetHeadType()
{
    uint8_t head_type;
    head_type = can_read_boardtype();
    switch (head_type)
    {
    case 0x00:
        SERIAL_ECHOLNPGM_P("error:No Print Head");
        break;
    case 0x01:
        HeadType = HEAD_TYPE_LASER;
        SERIAL_ECHOLNPGM_P("info:Head Laser");
        break;
    case 0x02:
        HeadType = HEAD_TYPE_CNC;
        SERIAL_ECHOLNPGM_P("info:Head CNC");
        break;
    case 0x03:
        HeadType = HEAD_TYPE_3DP;
        SERIAL_ECHOLNPGM_P("info:Head 3DP,0.3mm");
        break;
    default:
        SERIAL_ECHOLNPGM_P("error:No Print Type Unknown");
        break;
    }
}