#include "../inc/MarlinConfig.h"
#include "StatusManager.h"
#include "StopStepper.h"
#include "../gcode/queue.h"
#include "../gcode/gcode.h"
#include "Manager.h"
#include "../feature/bedlevel/bedlevel.h"
#include "DebugLog.h"

StatusManager SystemStatus;

void StatusManager::Init() {
    cur_status_ = SYSTAT_INIT;
    work_port_ = WORKING_PORT_NONE;
}

ErrCode StatusManager::PauseTrigger(TriggerSource type) {
    if (cur_status_ != SYSTAT_WORK && cur_status_!= SYSTAT_RESUME_WAITING) {
        DEBUG_W("Can't pause in current status: %d\n", cur_status_);
        return E_NO_SWITCHING_STA;
    }

    switch (type) {
        case TRIGGER_SOURCE_DOOR_OPEN:
            break;

        case TRIGGER_SOURCE_SC:
            if (work_port_ != WORKING_PORT_SC) {
                DEBUG_W("Current working port is not SCREEN!");
                return E_INVALID_STATE;
            }
            break;
        
        case TRIGGER_SOURCE_PC:
            if (work_port_ != WORKING_PORT_PC) {
                DEBUG_W("Current working port is not PC!");
                return E_INVALID_STATE;
            }
            break;

        default:
            DEBUG_W("Invlaid trigger source %d\n", type);
            return E_PARAM;
    }

    cur_status_ = SYSTAT_PAUSE_TRIG;

    stopstepper.Trigger(SS_EVENT_PAUSE);

    pause_source_ = type;

    return E_SUCCESS;
}

ErrCode StatusManager::StopTrigger(TriggerSource type) {
    if (cur_status_ != SYSTAT_WORK && cur_status_ != SYSTAT_RESUME_WAITING &&
        cur_status_ != SYSTAT_PAUSE_FINISH) {
        DEBUG_E("cannot stop in current status[%d]\n", cur_status_);
        return E_NO_SWITCHING_STA;
    }

    switch(type) {
    case TRIGGER_SOURCE_SC:
        if (work_port_ != WORKING_PORT_SC) {
        DEBUG_E("current working port is not SCREEN!");
        return E_FAILURE;
        }
        break;

    case TRIGGER_SOURCE_PC:
        if (work_port_ != WORKING_PORT_PC) {
        DEBUG_E("current working port is not PC!");
        return E_FAILURE;
        }
        break;

    case TRIGGER_SOURCE_FINISH:
        // if screen tell us Gcode is ended, wait for all movement output
        // because planner.synchronize() will call HMI process nestedly,
        // and maybe some function will check the status, so we change the status
        // firstly
        cur_status_ = SYSTAT_END_TRIG;
        planner.synchronize();
        break;

    default:
        break;
    }

    // if we already finish quick stop, just change system status
    // disable power-loss data, and exit with success
    if (cur_status_ == SYSTAT_PAUSE_FINISH) {
        // to make StopProcess work, cur_status_ need to be SYSTAT_END_FINISH
        cur_status_ = SYSTAT_END_FINISH;
        pause_source_ = TRIGGER_SOURCE_NONE;
        DEBUG_I("Stop in pauseing, trigger source: %d\n", type);
        return E_SUCCESS;
    }

    cur_status_ = SYSTAT_END_TRIG;

    stop_source_ = type;

    if (HeadManager.HeadType == HEAD_TYPE_LASER) {
        HeadManager.Laser.Off();
    }

    stopstepper.Trigger(SS_EVENT_STOP);

    return E_SUCCESS;
}

ErrCode StatusManager::ResumeTrigger(TriggerSource s) {
    if (cur_status_ != SYSTAT_PAUSE_FINISH) {
        DEBUG_W("Can't trigger resume in current status: %d\n", cur_status_);
        return E_NO_SWITCHING_STA;
    }

    switch (HeadManager.HeadType)
    {
    case HEAD_TYPE_3DP:
        /* code */
        break;

    case HEAD_TYPE_CNC:
        /* code */
        break;
    
    case HEAD_TYPE_LASER:
        /* code */
        break;
    
    default:
        break;
    }

    switch (s) {
    case TRIGGER_SOURCE_SC:
        if (work_port_ != WORKING_PORT_SC) {
            DEBUG_W("current working port is not SCREEN!");
            return E_FAILURE;
        }
        break;

    case TRIGGER_SOURCE_PC:
        if (work_port_ != WORKING_PORT_PC) {
            DEBUG_W("current working port is not PC!");
            return E_FAILURE;
        }
        break;

    case TRIGGER_SOURCE_DOOR_CLOSE:
        break;

    default:
        DEBUG_W("invalid trigger source: %d\n", s);
        return E_FAILURE;
        break;
    }

    cur_status_ = SYSTAT_RESUME_TRIG;

    return E_SUCCESS;
}

void StatusManager::ResumeProcess() {
    if (cur_status_ != SYSTAT_RESUME_TRIG)
        return;

    cur_status_ = SYSTAT_RESUME_MOVING;

    switch (HeadManager.HeadType)
    {
    case HEAD_TYPE_3DP:
        set_bed_leveling_enabled(true);
        Resume3dp();
        break;

    case HEAD_TYPE_CNC:
        ResumeCNC();
        delay(2000); //wait cnc open
        break;

    case HEAD_TYPE_LASER:
        ResumeLaser();
        break;
    
    default:
        break;
    }

    RestoreXYZ();

    queue.clear();

    pause_source_ = TRIGGER_SOURCE_NONE;
    cur_status_ = SYSTAT_RESUME_WAITING;
}

void StatusManager::PauseProcess() {
    if (GetCurrentStatus() != SYSTAT_PAUSE_STOPPED)
        return;

    cur_status_ = SYSTAT_PAUSE_FINISH;
    DEBUG_I("Finish pause\n");
}

void StatusManager::StopProcess() {
    if (cur_status_ != SYSTAT_END_FINISH)
        return;
    // clear stop type because stage will be changed
    stop_source_ = TRIGGER_SOURCE_NONE;
    pause_source_ = TRIGGER_SOURCE_NONE;
    cur_status_ = SYSTAT_IDLE;
    DEBUG_I("Finish stop\n");
}

void StatusManager::Process() {
    PauseProcess();
    StopProcess();
    ResumeProcess();
}

void inline StatusManager::Resume3dp(void) {

    enable_all_steppers();
    process_gcode("G28");
    process_gcode("G92 E0");

    gcode.set_relative_mode(true);
    process_gcode("G0 E20 F200");
    process_gcode("G0 E-6 F1800");

    process_gcode("G0 E6 F200");

    gcode.set_relative_mode(false);

    current_position[E_AXIS] = Data.PositionData[E_AXIS];
    sync_plan_position_e();

}

void inline StatusManager::ResumeCNC(void) {
    HeadManager.CNC.On();
}

void inline StatusManager::ResumeLaser(void) {
    HeadManager.Laser.On();
}

ErrCode StatusManager::StartWork(TriggerSource s) {
    if (cur_status_ != SYSTAT_IDLE) {
        DEBUG_W("cannot start work in current status: %d\n", cur_status_);
        return E_NO_SWITCHING_STA;
    }
    switch (HeadManager.HeadType)
    {
    case HEAD_TYPE_3DP:
        
        break;

    case HEAD_TYPE_CNC:
        HeadManager.CNC.On();
        break;
    
    default:
        break;
    }

    if (s == TRIGGER_SOURCE_SC) {
        //powerpanic.Data.GCodeSource = GCODE_SOURCE_SCREEN;
        work_port_ = WORKING_PORT_SC;
    }
    else if (s == TRIGGER_SOURCE_PC) {
        //powerpanic.Data.GCodeSource = GCODE_SOURCE_PC;
        work_port_ = WORKING_PORT_PC;
    }

    // set state
    cur_status_ = SYSTAT_WORK;
    return E_SUCCESS;
}

SysStage StatusManager::GetSystemStage() {
    switch (cur_status_) {
    case SYSTAT_INIT:
        return SYSTAGE_INIT;
        break;

    case SYSTAT_IDLE:
        return SYSTAGE_IDLE;
        break;

    case SYSTAT_WORK:
        return SYSTAGE_WORK;
        break;

    case SYSTAT_PAUSE_TRIG:
    case SYSTAT_PAUSE_STOPPED:
    case SYSTAT_PAUSE_FINISH:
        return SYSTAGE_PAUSE;
        break;

    case SYSTAT_RESUME_TRIG:
    case SYSTAT_RESUME_MOVING:
    case SYSTAT_RESUME_WAITING:
        return SYSTAGE_RESUMING;
        break;

    case SYSTAT_END_TRIG:
    case SYSTAT_END_FINISH:
        return SYSTAGE_END;
        break;

    default:
        return SYSTAGE_INVALID;
        break;
    }
}

void StatusManager::SaveCurrentPostion() {

    LOOP_XYZE(axis) {
        Data.PositionData[axis] = (axis == E_AXIS) ?
				current_position[axis] : NATIVE_TO_LOGICAL(current_position[axis], axis);
    }

}

void inline StatusManager::RestoreXYZ(void) {
    //DEBUG_I("\nrestore postion:\n X:%.2f, Y:%.2f, Z:%.2f)\n", Data.PositionData[X_AXIS], Data.PositionData[Y_AXIS], Data.PositionData[Z_AXIS]);
    do_blocking_move_to_logical_xy(Data.PositionData[X_AXIS], Data.PositionData[Y_AXIS], 60);
    planner.synchronize();

    if (HEAD_TYPE_CNC == HeadManager.HeadType) {
        do_blocking_move_to_logical_z(Data.PositionData[Z_AXIS] + 15, 30);
        do_blocking_move_to_logical_z(Data.PositionData[Z_AXIS], 10);
    }
    else {
        do_blocking_move_to_logical_z(Data.PositionData[Z_AXIS], 30);
    }
    planner.synchronize();
}