#include "../../inc/MarlinConfig.h"

#if ENABLED(IVI_LASER_CNC)
#include "../../gcode.h"
#include "../../../module/stepper.h"
#include "../../../module/Manager.h"
#include "../../../module/LaserManager.h"

void GcodeSuite::M3_M4(const bool is_M4) {
    planner.synchronize();
    
    if(HEAD_TYPE_LASER == HeadManager.HeadType) {
      if(parser.seen('S')) {
        HeadManager.Laser.SetPower(parser.value_float());
      }else {
        HeadManager.Laser.On();
      }
    }
//   else if(HEAD_TYPE_CNC == HeadManager.MachineType) {
//         if(parser.seen('S')) HeadManager.CNC.SetPower(parser.value_ushort());
//         else HeadManager.CNC.On();
//   }
}

void GcodeSuite::M5() {
    planner.synchronize();

    if(HEAD_TYPE_LASER == HeadManager.HeadType) {
        HeadManager.Laser.Off();
    }
    // else if(HEAD_TYPE_CNC == HeadManager.HeadType) {
    //     HeadManager.CNC.Off();
    // }
}

#endif // IVI_LASER_CNC