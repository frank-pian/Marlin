#include "../gcode.h"
#include "../../module/endstops.h"

//获取打印头类型

#define BOARD_TYPE_UNKNOWN	(0x00)
#define BOARD_TYPE_LASER	  (0x01)
#define BOARD_TYPE_CNC		  (0x02)
#define BOARD_TYPE_FDM30		(0x03)
#define BOARD_TYPE_FDM15		(0x04)
#define BOARD_TYPE_FDM20		(0x05)
#define BOARD_TYPE_FDM25		(0x06)
#define BOARD_TYPE_FDM35		(0x07)
#define BOARD_TYPE_FDM40		(0x08)

void GcodeSuite::M2051() {
  uint8_t board_type = 0;
  board_type = can_read_boardtype();
  if (board_type == 0x00) {  // error:No Print Head
    SERIAL_ECHOLNPGM_P("error:No Print Head");
  } else if (board_type == BOARD_TYPE_LASER) {  // info:Head Laser
    SERIAL_ECHOLNPGM_P("info:Head Laser");
  } else if (board_type == BOARD_TYPE_CNC) {  // info:Head CNC
    SERIAL_ECHOLNPGM_P("info:Head CNC");
  } else if (board_type == BOARD_TYPE_FDM30) {  // info:Head 3DP,0.3mm
    SERIAL_ECHOLNPGM_P("info:Head 3DP,0.3mm");
  } else if (board_type == BOARD_TYPE_FDM15) {  // info:Head 3DP,0.15mm
    SERIAL_ECHOLNPGM_P("info:Head 3DP,0.15mm");
  } else if (board_type == BOARD_TYPE_FDM20) {  // info:Head 3DP,0.2mm
    SERIAL_ECHOLNPGM_P("info:Head 3DP,0.2mm");
  } else if (board_type == BOARD_TYPE_FDM25) {  // info:Head 3DP,0.25mm
    SERIAL_ECHOLNPGM_P("info:Head 3DP,0.25mm");
  } else if (board_type == BOARD_TYPE_FDM35) {  // info:Head 3DP,0.35mm
    SERIAL_ECHOLNPGM_P("info:Head 3DP,0.35mm");
  } else if (board_type == BOARD_TYPE_FDM40) {  // info:Head 3DP,0.4mm
    SERIAL_ECHOLNPGM_P("info:Head 3DP,0.4mm");
  } else {
    SERIAL_ECHOLNPGM_P("error:No Print Type Unknown");
  }
}