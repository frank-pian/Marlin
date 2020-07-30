#include "../../inc/MarlinConfig.h"
#include "../gcode.h"
#include "../../module/StopStepper.h"

void GcodeSuite::M2410() 
{
    stopstepper.Trigger(SS_EVENT_PAUSE);
}