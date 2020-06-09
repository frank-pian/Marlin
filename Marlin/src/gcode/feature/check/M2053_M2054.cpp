#include "../../gcode.h"
#include "../../module/endstops.h"

//断堵料状态查询

void GcodeSuite::M2053() {

  SERIAL_ECHOLNPGM_P("warning:Tube No Material");

}

//断堵料编码器信息查询

void GcodeSuite::M2054() {

  SERIAL_ECHOLNPGM_P("info:80");

}