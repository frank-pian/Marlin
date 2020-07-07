#include "../gcode.h"

void GcodeSuite::M2005() {
    serial_echopair_PGM("Firmware Version:", SHORT_BUILD_VERSION);
    SERIAL_EOL();
}