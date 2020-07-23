#include "../gcode.h"
#include "../../module/PeripheManager.h"

//获取门开关状态

void GcodeSuite::M2010() {
  uint8_t s;
  if (!parser.seen('S')) {
    Periphe.DoorCheck();
    return;
  }
  s = parser.byteval('S', 12);
  switch (s){
    case 0:
      Periphe.SetDoorCheckStatus(false);
      break;

    case 1:
      Periphe.SetDoorCheckStatus(true);
      break;
      
    default:
      break;
  }
}