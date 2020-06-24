#include "../gcode.h"
#include "../../module/endstops.h"

//获取重量传感器震动值

void GcodeSuite::M2049() {
  float temp0 = 20, temp1 = 0;
  temp0 = hx710_analog_to_weigh(5);
  //SERIAL_ECHOLNPGM_P("info:80");
  serial_echopair_PGM(" info:weight_0 ", temp0);
  serial_echopair_PGM(" info:weight_1 ", temp1);
  SERIAL_EOL();
}