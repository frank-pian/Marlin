#include "../gcode.h"
#include "../../module/endstops.h"

//获取打印头类型

#define BOARD_TYPE_UNKNOWN	(0x00)
#define BOARD_TYPE_LASER	(0x01)
#define BOARD_TYPE_CNC		(0x02)
#define BOARD_TYPE_FDM		(0x03)


void GcodeSuite::M2051() {
  uint8_t board_type = 0;
  board_type = can_read_boardtype();
  if (board_type == 0x00) {  // error:No Print Head
    SERIAL_ECHOLNPGM_P("error:No Print Head");
  } else if (board_type == BOARD_TYPE_LASER) {  // info:Head Laser
    SERIAL_ECHOLNPGM_P("info:Head Laser");
  } else if (board_type == BOARD_TYPE_CNC) {  // info:Head CNC
    SERIAL_ECHOLNPGM_P("info:Head CNC");
  } else if (board_type == BOARD_TYPE_FDM) {  // info:Head 3DP,0.3mm
    SERIAL_ECHOLNPGM_P("info:Head 3DP,0.3mm");
  } else {
    SERIAL_ECHOLNPGM_P("error:No Print Type Unknown");
  }
}