/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * IVI 3D Controller (STM32F103VET6) board pin assignments
 */

#if !defined(__STM32F1__) && !defined(STM32F1xx)
  #error "Oops! Select a STM32F1 board in 'Tools > Board.'"
#elif HOTENDS > 2 || E_STEPPERS > 2
  #error "IVI3D board only supports 2 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_NAME "IVI3D"

// #define DISABLE_DEBUG
#define DISABLE_JTAG
//
// Limit Switches
//
#define X_MIN_PIN          PE7   // pin 16
#define X_MAX_PIN          PE7   // pin 15 (Filament sensor on Alfawise setup)
#define Y_MIN_PIN          PE1  // pin 9
#define Y_MAX_PIN          PE1  // 
#define Z_MIN_PIN          PE3   // Standard Endstop or Z_Probe endstop function
#define Z_MAX_PIN          PE2   // pin 4 (Unused in stock Alfawise setup)
                                 // May be used for BLTouch Servo function on older variants (<= V08)

//
// Filament Sensor
//
// #ifndef FIL_RUNOUT_PIN
//   #define FIL_RUNOUT_PIN   PC0   // XMAX plug on PCB used as filament runout sensor on Alfawise boards (inverting true)
// #endif

//
// Steppers
//
#define X_ENABLE_PIN       PC6   // pin 
#define X_STEP_PIN         PD14   // pin 
#define X_DIR_PIN          PD15   // pin 

#define Y_ENABLE_PIN       PD12   // pin 
#define Y_STEP_PIN         PD10   // pin 
#define Y_DIR_PIN          PD11   // pin 

#define Z_ENABLE_PIN       PD8   // pin 
#define Z_STEP_PIN         PE14   // pin 
#define Z_DIR_PIN          PE15   // pin 

#define E0_ENABLE_PIN      PE13   // pin 
#define E0_STEP_PIN        PE11   // pin 
#define E0_DIR_PIN         PE12   // pin 

#define E1_ENABLE_PIN      PE10   // pin 
#define E1_STEP_PIN        PE8   // pin 
#define E1_DIR_PIN         PE9   // pin 
//
// Temperature Sensors
//
#define TEMP_0_PIN         PB0   // pin 23 (Nozzle 100K/3950 thermistor)
#define TEMP_BED_PIN       PA0   // pin 24 (Hot Bed 100K/3950 thermistor)

//
// Heaters / Fans
//
#define HEATER_0_PIN       PA1   // pin 84 (Nozzle Heat Mosfet)
#define HEATER_BED_PIN     PA2   // pin 67 (Hot Bed Mosfet)

#define FAN_PIN            PC2  // pin 77 (4cm Fan)
#define FAN1_PIN           PC0
#define FAN2_PIN           PC1
#define FAN_SOFT_PWM             // Required to avoid issues with heating or STLink
#define FAN_MIN_PWM        35    // Fan will not start in 1-30 range
#define FAN_MAX_PWM        255

//#define BEEPER_PIN       PD13  // pin 60 (Servo PWM output 5V/GND on Board V0G+) made for BL-Touch sensor
                                 // Can drive a PC Buzzer, if connected between PWM and 5V pins

#define LED_PIN            PB2   // 

#define SPINDLE_LASER_ENA_PIN    PD7   // digital pin
#define SPINDLE_LASER_PWM_PIN    PA7   // digital pin - MUST BE HARDWARE PWM ; TIM4_CH2
#define SPINDLE_DIR_PIN          PB3   // digital pin

#define DOGLCD_MOSI        -1  // Prevent auto-define by Conditionals_post.h
#define DOGLCD_SCK         -1

//
// Persistent Storage
// If no option is selected below the SD Card will be used
//
//#define SPI_EEPROM
#define FLASH_EEPROM_EMULATION

#define POWER_LOSS_PIN     PA3

#undef E2END
#if ENABLED(SPI_EEPROM)
  // SPI1 EEPROM Winbond W25Q64 (8MB/64Mbits)
  #define SPI_CHAN_EEPROM1   1
  #define SPI_EEPROM1_CS     PC5   // pin 34
  #define EEPROM_SCK  BOARD_SPI1_SCK_PIN    // PA5 pin 30
  #define EEPROM_MISO BOARD_SPI1_MISO_PIN   // PA6 pin 31
  #define EEPROM_MOSI BOARD_SPI1_MOSI_PIN   // PA7 pin 32
  #define EEPROM_PAGE_SIZE   0x1000U        // 4KB (from datasheet)
  #define E2END ((16 * EEPROM_PAGE_SIZE)-1) // Limit to 64KB for now...
#elif ENABLED(FLASH_EEPROM_EMULATION)
  // SoC Flash (framework-arduinoststm32-maple/STM32F1/libraries/EEPROM/EEPROM.h)
  #define EEPROM_START_ADDRESS (0x8000000UL + (512 * 1024) - 2 * EEPROM_PAGE_SIZE)
  #define EEPROM_PAGE_SIZE     (0x800U)     // 2KB, but will use 2x more (4KB)
  #define E2END (EEPROM_PAGE_SIZE - 1)
#else
  #define E2END (0x7FFU) // On SD, Limit to 2KB, require this amount of RAM
#endif

// senser pin define

#define EXTENSION_SENSERS

#if ENABLED(EXTENSION_SENSERS)

  #define DHTTYPE         DHT11   // DHT 11
  #define DHTPIN          PA15     // Digital pin connected to the DHT sensor
  #define ONE_WIRE_PIN    PB1
  #define DOOR_PIN        PA5

  #if EXTRUDERS
    #define FEED1_PIN     PD9
  #elif EXTRUDERS > 1
    #define FEED2_PIN     PD13
  #endif

#endif

/**
 *  HX711 Filament weight sensing
 */
#define HAVE_HX711
#if ENABLED(HAVE_HX711)

  #if EXTRUDERS
    #define WEIGHT0_DOUT_PIN        PE4
    #define WEIGHT0_SCK_PIN         PE5
    #define EXTRUDER0_DOUT_PIN      PB5
    #define EXTRUDER0_SCK_PIN       PC7
  #elif EXTRUDERS > 1
    #define WEIGHT1_DOUT_PIN        PA6
    #define WEIGHT1_SCK_PIN         PA7
    #define EXTRUDER1_DOUT_PIN      PD4
    #define EXTRUDER1_SCK_PIN       PD3
  #endif

  #define HX711_CALIBRATION_FACTOR    -1 // Initial set_scale calibration value.
  
  #define HX711_CHECKINTERVAL_MILLIS  5000  // HX711 check interval in milliseconds.
  extern void hx711_init1();
  extern void dht11_init();
  extern void eprom_init();
  extern void auto_report_weight();
  extern void auto_report_extruder();
  extern void auto_report_senser();

#endif
