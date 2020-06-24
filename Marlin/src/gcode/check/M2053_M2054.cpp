#include "../gcode.h"
#include "../../module/endstops.h"

//断堵料状态查询

void GcodeSuite::M2053() {
  if (encoder_get_statue())
    SERIAL_ECHOLNPGM_P("warning:Tube Abnormal");
  else
    SERIAL_ECHOLNPGM_P("info:ok");
}

//断堵料编码器信息查询

void GcodeSuite::M2054() {
  int count = encoder_get_value();
  serial_echopair_PGM("info:", count);
  SERIAL_EOL();
}