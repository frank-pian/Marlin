#include "../../gcode.h"
#include "../../module/endstops.h"

//获取门开关状态

void GcodeSuite::M1007() {

  //门打开
  SERIAL_ECHOLNPGM_P("warning:Door Openned");

  //门关闭
  //SERIAL_ECHOLNPGM_P("warning:Door Closed");

}