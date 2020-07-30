#ifndef STOPSTEPPER_H_
#define STOPSTEPPER_H_

#include "./error.h"
#include "./planner.h"

#define CNC_SAFE_HIGH_DIFF 30  // Bed to CNC head height. mm

enum StopStepperEvent : uint8_t {
    SS_EVENT_NONE = 0,
    SS_EVENT_PAUSE,           // pause working, filament runout, door open
    SS_EVENT_STOP,            // stop/finish working, button stop
    SS_EVENT_ISR_POWER_LOSS,  // power loss
    SS_EVENT_INVALID
};

enum StopStepperSync : uint8_t {
    SS_SYNC_NONE = 0,
    SS_SYNC_TRIGGER,
    SS_SYNC_ISR_END,
    SS_SYNC_INVALID
};

class StopStepper
{
    public:
        bool CheckISR(block_t *blk);
        ErrCode Trigger(StopStepperEvent e);
        void Process();

    private:
        volatile StopStepperEvent event_ = SS_EVENT_NONE;
        volatile StopStepperSync sync_flag_ = SS_SYNC_NONE;
        bool disable_stepper_ = false;
        void CleanMoves();
        void TowardStop();
};

extern StopStepper stopstepper;

#endif // #ifndef STOPSTEPPER_H_