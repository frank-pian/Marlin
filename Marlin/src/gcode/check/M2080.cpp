#include "../gcode.h"
#include "../../module/endstops.h"

//获取加速度传感器震动值

void GcodeSuite::M2080() {
  uint8_t temp = can_read_mpu6500();

  //SERIAL_ECHOLNPGM_P("info:80");
  serial_echopair_PGM("info:", temp);
  SERIAL_EOL();
}