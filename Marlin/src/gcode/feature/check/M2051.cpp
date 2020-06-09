#include "../../gcode.h"
#include "../../module/endstops.h"

//获取打印头类型

void GcodeSuite::M2051() {

  SERIAL_ECHOLNPGM_P("info:Head 3DP,0.3mm");

}