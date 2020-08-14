#pragma once

#include "../inc/MarlinConfig.h"
#include "error.h"

#ifndef _STATUS_MANAGER_H_
#define _STATUS_MANAGER_H_

enum TriggerSource : uint8_t {
  TRIGGER_SOURCE_NONE,
  TRIGGER_SOURCE_SC,              // trigger by screen
  TRIGGER_SOURCE_PC,              // trigger by PC
  TRIGGER_SOURCE_RUNOUT,          // trigger by filament runout
  TRIGGER_SOURCE_DOOR_OPEN,       // trigger by door opened
  TRIGGER_SOURCE_DOOR_CLOSE,      // trigger by door closed
  TRIGGER_SOURCE_STOP_BUTTON,     // trigger by emergency button
  TRIGGER_SOURCE_FINISH,          // trigger by job finished
  TRIGGER_SOURCE_EXCEPTION,       // trigger by exception

  TRIGGER_SOURCE_INVALID
};

// status of system
enum SysStatus: uint8_t {
  SYSTAT_INIT = 0,

  SYSTAT_IDLE,            // generally, we are in this status after initialization

  SYSTAT_WORK,            // working

  SYSTAT_PAUSE_TRIG,      // pause is triggered by some events
  SYSTAT_PAUSE_STOPPED,   // quick stop is handled
  SYSTAT_PAUSE_FINISH,    // the continuous status for pause

  SYSTAT_RESUME_TRIG,     // resume op is triggered
  SYSTAT_RESUME_MOVING,   // moving to the position when pause
  SYSTAT_RESUME_WAITING,  // waiting the Gcode

  SYSTAT_END_TRIG,
  SYSTAT_END_FINISH,
  SYSTAT_INVALID
};

enum SysStage : uint8_t {
  SYSTAGE_INIT,
  SYSTAGE_IDLE,
  SYSTAGE_WORK,
  SYSTAGE_PAUSE,
  SYSTAGE_RESUMING,
  SYSTAGE_END,
  SYSTAGE_INVALID
};

enum WorkingPort : uint8_t {
  WORKING_PORT_NONE,  // no work started from any port
  WORKING_PORT_PC,    // started a work from PC port
  WORKING_PORT_SC,    // started a work from Screen port
  WORKING_PORT_INVALID
};

typedef struct __attribute__((aligned (4)))
{
  xyze_pos_t PositionData;
} StatusSaveData;

class StatusManager {

    public:
        StatusSaveData Data;

        ErrCode PauseTrigger(TriggerSource type);
        ErrCode StopTrigger(TriggerSource type);
        ErrCode ResumeTrigger(TriggerSource s);

        StatusManager(){};
        void Init();
        void Process();
        ErrCode StartWork(TriggerSource s);
        SysStage GetSystemStage();
        void SaveCurrentPostion();


        SysStatus FORCE_INLINE GetCurrentStatus() {
            return cur_status_;
        }

        ErrCode FORCE_INLINE SetCurrentStatus(SysStatus s) {
            if (s < SYSTAT_INVALID) {
                cur_status_ = s;
                return E_SUCCESS;
            }

            return E_PARAM;
        }


    private:
        TriggerSource pause_source_;  // record latest pause source
        TriggerSource stop_source_;
        SysStatus cur_status_;
        WorkingPort work_port_;    // indicates we are handling Gcode from which UART

        void ResumeProcess();
        void PauseProcess();
        void StopProcess();
        void inline RestoreXYZ(void);

        void inline Resume3dp(void);
        void inline ResumeCNC(void);
        void inline ResumeLaser(void);
};


extern StatusManager SystemStatus;

#endif //def _STATUS_MANAGER_H_