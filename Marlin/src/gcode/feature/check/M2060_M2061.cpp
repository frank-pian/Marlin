#include "../../gcode.h"
#include "../../module/endstops.h"

void GcodeSuite::M2060() {

  SERIAL_ECHOLNPGM_P("sensor: XMotor:ok YMotor:ok ZMotor:ok XLimit:ok YLimit:ok ZLimit:ok Fans:ok Door:ok TempHumi:ok ToolHead:ok");

}

void GcodeSuite::M2061() {

  SERIAL_ECHOLNPGM_P("sensor: Print:ok PrintHeat:ok Bed:ok BedHeat:ok Extruder:ok Filament:ok Auto:ok");
  // SERIAL_ECHOLNPGM_P("sensor: Laser:ok LaserTemp:ok");
  // SERIAL_ECHOLNPGM_P("sensor: CNC:ok CNCTemp:ok");
}