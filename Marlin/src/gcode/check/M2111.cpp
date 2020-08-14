
#include "../../module/DebugLog.h"
#include "../gcode.h"
#include "../queue.h"
#include "../../core/macros.h"
#include "../../module/motion.h"
#include "../../module/StatusManager.h"

#if HAS_POSITION_SHIFT
  // The distance that XYZ has been offset by G92. Reset by G28.
  extern xyz_pos_t position_shift;
#endif
// #if HAS_HOME_OFFSET
//   // This offset is added to the configured home position.
//   // Set by M206, M428, or menu item. Saved to EEPROM.
//   extern float home_offset[XYZ];
// #endif
#if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
  // The above two are combined to save on computes
  extern float workspace_offset[XYZ];
#endif

extern xyze_pos_t current_position;

void GcodeSuite::M2111() {
  uint8_t l;
  uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0);

  switch (s) {
  case 0:
    // show current info
    //DEBUG_SHOW_INFO();
    DEBUG_I("position_shift:\n");
    DEBUG_I("X: %f, Y:%f, Z:%f\n", position_shift[X_AXIS], position_shift[Y_AXIS], position_shift[Z_AXIS]);
    // DEBUG_I("home_offset:\n");
    // DEBUG_I("X: %f, Y:%f, Z:%f\n", home_offset.x, home_offset.y, home_offset.z);
    // DEBUG_I("workspace_offset:\n");
    // DEBUG_I("X: %f, Y:%f, Z:%f\n", workspace_offset[X_AXIS], workspace_offset[Y_AXIS], workspace_offset[Z_AXIS]);
    DEBUG_I("cur position:\n");
    DEBUG_I("X: %f, Y:%f, Z:%f\n", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
    break;

  case 1:
    // set PC log level
    l = (uint8_t)parser.byteval('L', (uint8_t)10);
    if (!WITHIN(l, 0, (int)DEBUG_LEVEL_MAX)) {
      DEBUG_E("L out of range (0-%d)\n", (int)DEBUG_LEVEL_MAX);
      return;
    }
    DEBUG_SET_LEVEL(0, (DebugLevel)l);

    break;
  case 2:
    // set SC log level
    l = (uint8_t)parser.byteval('L', (uint8_t)10);
    if (!WITHIN(l, 0, (int)DEBUG_LEVEL_MAX)) {
      DEBUG_E("L out of range (0-%d)\n", (int)DEBUG_LEVEL_MAX);
      return;
    }
    DEBUG_SET_LEVEL(1, (DebugLevel)l);
    break;

//   case 3:
//     DEBUG_SHOW_EXCEPTION();
//     break;

//   case 4:
//     l = (uint8_t)parser.byteval('L', (uint8_t)0);
//     if (!WITHIN(l, 1, 32)) {
//       DEBUG_E("L is out of range (1-32)\n");
//       return;
//     }
//     SystemStatus.ClearExceptionByFaultFlag(1<<(l-1));
//     break;
   }

}