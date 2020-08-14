#include "../../inc/MarlinConfig.h"
#include "../gcode.h"
#include "../../module/StatusManager.h"
#include "../../module/DebugLog.h"

void GcodeSuite::M2410() {
    // ErrCode err;
    // DEBUG_I("PC trigger pause\n");
    // err = SystemStatus.PauseTrigger(TRIGGER_SOURCE_PC);
    // if (err == E_SUCCESS) {
    //     DEBUG_I("trigger pause: ok\n");
    // }else {
    //     DEBUG_W("trigger pause failed, err = %d\n", err);
    // }
}

void GcodeSuite::M2411() {
    ErrCode err;
    uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0);

    switch (s)
    {

    case 1:
        //RESUME WORK
        DEBUG_I("PC trigger resume\n");
        err = SystemStatus.ResumeTrigger(TRIGGER_SOURCE_PC);
        if (err == E_SUCCESS) {
            DEBUG_I("trigger resume: ok\n");
        }else {
            DEBUG_W("trigger RESUME: failed, err = %d\n", err);
        }
        break;

    case 2:
        //START WORK
        DEBUG_I("PC START WORK\n");
        err = SystemStatus.StartWork(TRIGGER_SOURCE_PC);
        if (E_SUCCESS == err) {
            DEBUG_I("trigger WORK: ok\n");
        }else {
            DEBUG_W("failed to start work: err= %d\n", err);
        }
        break;

    case 3:
        //STOP WORK
        DEBUG_I("PC trigger STOP\n");
        err = SystemStatus.StopTrigger(TRIGGER_SOURCE_PC);
        if (err == E_SUCCESS) {
            DEBUG_I("trigger STOP: ok\n");
        }else {
            DEBUG_W("trigger STOP: failed, err = %d\n", err);
        }
        break;

    case 4:
        //FINISH WORK
        DEBUG_I("pc trigger FINISH\n");
        err = SystemStatus.StopTrigger(TRIGGER_SOURCE_FINISH);
        if (err != SYSTAT_WORK) {
            DEBUG_I("trigger FINISH: ok\n");
        }else {
            DEBUG_W("trigger FINISH: failed, err = %d\n", err);
        }
    
    default:
        break;
    }
    
}
