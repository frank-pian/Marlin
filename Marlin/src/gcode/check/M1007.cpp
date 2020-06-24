#include "../gcode.h"
#include "../../module/endstops.h"

//获取门开关状态

void GcodeSuite::M1007() {
  //门打开
  if (READ(DOOR_STATUS_PIN))
    SERIAL_ECHOLNPGM_P("warning:Door Openned");

  //门关闭
  if (!READ(DOOR_STATUS_PIN))
    SERIAL_ECHOLNPGM_P("warning:Door Closed");

}