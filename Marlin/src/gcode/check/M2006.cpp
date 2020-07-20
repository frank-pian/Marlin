#include "../gcode.h"

void GcodeSuite::M2006() {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM("Auto Fan Switch: ");
    if (parser.seen('S')) {
        auto_fan_switch = parser.value_bool();
    }
    serialprint_onoff(auto_fan_switch);
    SERIAL_EOL();
}