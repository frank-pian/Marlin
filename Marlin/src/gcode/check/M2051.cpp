#include "../gcode.h"
#include "../../module/Manager.h"

//获取打印头类型


void GcodeSuite::M2051() {
  HeadManager.GetHeadType();
}