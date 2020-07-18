#include "../gcode.h"
#include "../../module/configuration_store.h"
#include "../../HAL/STM32/stm32_eeprom_flash.h"

extern void reset_eeprom_written(void);

void GcodeSuite::M2050() {
  change_to_factory_address();
  (void)settings.save();

  reset_eeprom_written();
  change_to_user_address();
  (void)settings.save();

  // SERIAL_ECHOLNPGM_P("OK");
  // SERIAL_EOL();
}