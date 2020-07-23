/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2017 Victor Perez
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
#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "HAL.h"
#include "usb_serial.h"

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

#if ENABLED(SRAM_EEPROM_EMULATION)
  #if STM32F7xx
    #include <stm32f7xx_ll_pwr.h>
  #elif STM32F4xx
    #include <stm32f4xx_ll_pwr.h>
  #else
    #error "SRAM_EEPROM_EMULATION is currently only supported for STM32F4xx and STM32F7xx"
  #endif
#endif

// ------------------------
// Public Variables
// ------------------------

uint16_t HAL_adc_result;  

// ------------------------
// Public functions
// ------------------------

void SHT20I2C_Init(void);
void CAN2_Init(void);
void misc_pin_init(void);

// Needed for DELAY_NS() / DELAY_US() on CORTEX-M7
#if (defined(__arm__) || defined(__thumb__)) && __CORTEX_M == 7
  // HAL pre-initialization task
  // Force the preinit function to run between the premain() and main() function
  // of the STM32 arduino core
  __attribute__((constructor (102)))
  void HAL_preinit() {
    enableCycleCounter();
  }
#endif

// HAL initialization task
void HAL_init() {
  FastIO_init();

  #if ENABLED(SDSUPPORT)
    OUT_WRITE(SDSS, HIGH); // Try to set SDSS inactive before any other SPI users start up
  #endif

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  #if ENABLED(SRAM_EEPROM_EMULATION)
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();           // Enable access to backup SRAM
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    LL_PWR_EnableBkUpRegulator();         // Enable backup regulator
    while (!LL_PWR_IsActiveFlag_BRR());   // Wait until backup regulator is initialized
  #endif

  SetSoftwareSerialTimerInterruptPriority();

  SHT20I2C_Init();
  CAN2_Init();
  misc_pin_init();
  // can_read_boardtype();
}

void HAL_clear_reset_source() { __HAL_RCC_CLEAR_RESET_FLAGS(); }

uint8_t HAL_get_reset_source() {
  return
    #ifdef RCC_FLAG_IWDGRST // Some sources may not exist...
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)  ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_IWDG1RST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_IWDG2RST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDG2RST) ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_SFTRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)   ? RST_SOFTWARE :
    #endif
    #ifdef RCC_FLAG_PINRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)   ? RST_EXTERNAL :
    #endif
    #ifdef RCC_FLAG_PORRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)   ? RST_POWER_ON :
    #endif
    0
  ;
}

void _delay_ms(const int delay_ms) { delay(delay_ms); }

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// ------------------------
// ADC
// ------------------------

// TODO: Make sure this doesn't cause any delay
void HAL_adc_start_conversion(const uint8_t adc_pin) { HAL_adc_result = analogRead(adc_pin); }

uint16_t HAL_adc_get_result() { return HAL_adc_result; }

void flashFirmware(const int16_t) { NVIC_SystemReset(); }

/* HX710 */
// GPIOA_6 DATA
// GPIOA_7 CLK

void us_delay(uint32_t us)
{
	volatile uint32_t cnt = 5 * us;
	while(cnt--);
}

uint32_t PD_DAT_RD(void)	
{
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT);
	return LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6);
}

void hx711_wait_ready(void)
{
  int temp = 0;
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	
	while(PD_DAT_RD()) {
		us_delay(1);
    temp++;
    if (temp > 200000)
      break;
  }
}

void PD_SCK(int val)
{
	if (val)
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
	else
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
}

uint32_t get_hx711_value(void)
{
	int i = 0;
	uint32_t count = 0;
	hx711_wait_ready();
	PD_SCK(1);
	us_delay(1);
	for (i = 0; i < 24; i++) {
		PD_SCK(1);
		us_delay(10);
		PD_SCK(0);
    us_delay(3);
		count <<= 1;
		if (PD_DAT_RD())
			count++;
		us_delay(10);
	}
	PD_SCK(1);
	us_delay(10);
	count = count ^ 0x800000;
	PD_SCK(0);
	return count;
}

float hx710_analog_to_weigh(uint32_t time)
{
  uint32_t i = 0;
  float weight = 0.0;
  uint32_t weight_sum = 0;
  if (time == 0) {
    return 0;
  } else if (time > 20) {
    time = 20;
  }

  for (i = 0; i < time; i++)
  {
    weight_sum += get_hx711_value();
  }

  // weight = ((float)weight_sum) / ((float)time) * 0.00053 - 284; // 2KG bridge
  weight = ((float)weight_sum) / ((float)time) * 0.000919 - 7700.0; // 1KG bridge

  return weight;
}

/***********************************************************************************************/
/* *********** SHTC20 *********** */
#define MEAS_T_HOLD       0xe3
#define MEAS_RH_HOLD      0xe5
#define MEAS_T_LOOP       0xf3
#define MEAS_RH_LOOP      0xf5
#define WRITE_USER_REG    0xe6
#define READ_USER_REG     0xe7
#define SOFT_RESET        0xfe

#define SHT20_WR_ADDR			(0x80)
#define SHT20_RD_ADDR			(0x81)

// SCL <---> PA8
// SDA <---> PC9
void xus_delay(uint32_t us)
{
	volatile uint32_t cnt = 50 * us;
	while(cnt--);
}

#define I2C_DELAY		xus_delay(10)
// #define ms_delay(X)    delay(X)
#define ms_delay(X)    xus_delay(1000*X)
#define I2C_SDA_IN  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT)
#define I2C_SDA_OUT LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT)

#define I2C_SCL_H		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8)
#define I2C_SCL_L		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8)

#define I2C_SDA_H		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9)
#define I2C_SDA_L		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9)

#define I2C_SDA_READ 	LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_9)

// i2c initialization
void SHT20I2C_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void I2C_Send_Start(void)
{
	I2C_SDA_OUT;
	I2C_SDA_H;
	I2C_DELAY;
	I2C_SCL_H;
	I2C_DELAY;
	
	I2C_SDA_L;
	I2C_DELAY;
}
void I2C_Send_stop(void)
{
	I2C_SDA_OUT;
	I2C_SDA_L;
	I2C_DELAY;
	I2C_SCL_H;
	I2C_DELAY;
	I2C_SDA_H;
}

void I2C_Ack(void)
{
	I2C_SDA_OUT;
	I2C_SCL_L;
	I2C_DELAY;
	I2C_SDA_L;
	I2C_DELAY;
	I2C_SCL_H;
	I2C_DELAY;
	I2C_SCL_L;
}

void I2C_NoAck(void)
{
	I2C_SDA_OUT;
	I2C_SCL_L;
	I2C_DELAY;
	I2C_SDA_H;
	I2C_DELAY;
	I2C_SCL_H;
	I2C_DELAY;
	I2C_SCL_L;
}


uint8_t I2C_WaitAck(void)
{
	int time = 1000;
	uint8_t ack = 0;
	I2C_SDA_IN;
//	I2C_SDA_H;
	I2C_DELAY;
	I2C_SCL_H;
	I2C_DELAY;
	
	while(time--) {
		if (!I2C_SDA_READ) {
			ack = 1;
			break;
		}
	}
	
	I2C_DELAY;
	I2C_SCL_L;
	return ack;
}

uint32_t I2C_Send_byte(uint8_t data, uint8_t ack)
{
	int i = 0;
	uint32_t ret = 0;
	uint8_t temp = data;
	
	I2C_SDA_OUT;
	I2C_DELAY;
	for (i = 0; i < 8; i++) {
		I2C_SCL_L;
		I2C_DELAY;
		if (temp & 0x80) I2C_SDA_H;
		else I2C_SDA_L;
		I2C_DELAY;
		I2C_SCL_H;
		I2C_DELAY;
		temp <<= 1;
	}
	I2C_SCL_L;
	I2C_DELAY;
	
	if (ack) {
		ret = I2C_WaitAck();
	} else {
		I2C_SDA_H;
		I2C_DELAY;
		I2C_SCL_H;
		I2C_DELAY;
		I2C_SCL_L;
	}

	return ret;
}

uint8_t I2C_Read_byte(uint8_t ack)
{
	int i = 0;
	uint8_t value = 0;
	
	I2C_SDA_IN;
	I2C_DELAY;
	for (i = 0; i < 8; i++) {
		I2C_SCL_L;
		I2C_DELAY;
		I2C_SCL_H;
		
		value <<= 1;
		if (I2C_SDA_READ)
			value |= 1;
		I2C_DELAY;
	}
	
	if (ack) {
		I2C_Ack();
	} else {
		I2C_NoAck();
	}
	
	return value;
}

uint32_t sht20_read(uint8_t cmd)
{
	int i = 0;
	uint32_t temp = 0;
	uint32_t checksum = 0;
	I2C_Send_Start();
	if (!I2C_Send_byte(SHT20_WR_ADDR, 1)) {
		I2C_Send_stop();
		return 0;
	}
	I2C_Send_byte(cmd, 1);
	us_delay(100);
	I2C_Send_stop();
	ms_delay(10);
	
	i = 10;
	while(i--) {
		I2C_Send_Start();
		if (I2C_Send_byte(SHT20_RD_ADDR, 1)){
			// read data
			temp = (uint32_t)I2C_Read_byte(1) << 8;
			temp |= (uint32_t)I2C_Read_byte(1);
			checksum = I2C_Read_byte(0);
			I2C_Send_stop();
			break;
		} else {
			I2C_Send_stop();
		}
		ms_delay(2);
	}
  (void)checksum;
	return temp;
}

void sht20_reset(void)
{
	I2C_Send_Start();
	if (!I2C_Send_byte(SHT20_WR_ADDR, 1)) {
		I2C_Send_stop();
	}
	I2C_Send_byte(0xfe, 1);
	us_delay(50);
	I2C_Send_stop();
	ms_delay(100);
}

float temperature = 0;
float humidity = 0;

void sht20_update(void)
{
  temperature = sht20_read(0xf3);
	temperature = -46.85 + 175.72 * temperature / 65536;
//  ms_delay(50);
  humidity = sht20_read(0xf5);
	humidity = -6 + 125 * humidity / 65536;
}

void sht20_get_value(float *temp, float *hum)
{
  *temp = temperature;
  *hum = humidity;
}

/****************************************************************************/
// can bus
#include "stm32f4xx_hal_can.h"

#define CAN_TIMEOUT   (10)

CAN_RxHeaderTypeDef rx_msg;
uint8_t rx_data[8] = {0};
CAN_HandleTypeDef hcan2;
bool can_enable = true;

uint8_t can_timeout = 0;
uint8_t board_type_flag = 0;
uint8_t z_pro_flag = 0;
uint8_t z_pro_int_flag = 0;
uint8_t status_flag = 0;
uint8_t head_temperature_flag = 0;
uint8_t mpu6500_flag = 0;

volatile uint8_t z_pro = 0;
uint8_t z_pro_int = 0;
uint8_t status = 0;
uint8_t board_type = 0;
float head_temperature = 0;
int16_t mpu6500[3] = {0};

bool auto_fan_switch = true;

void CAN2_Init(void)
{
	CAN_FilterTypeDef CAN_FilterInitStructure = {0};

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL; //;CAN_MODE_LOOPBACK
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  
  CAN_FilterInitStructure.FilterBank = 0;
  CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterInitStructure.FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.FilterIdLow = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  CAN_FilterInitStructure.FilterActivation = CAN_FILTER_ENABLE;
  HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterInitStructure);
  
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
    return;
  
  // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  // NVIC_SetPriority(CAN2_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  // NVIC_EnableIRQ(CAN2_RX0_IRQn);
  // HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  can_enable = true;
}

void can_parser(void);
void CAN2_RX0_IRQHandler(void)
{
  // HAL_CAN_IRQHandler(&hcan2);
  // HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_msg, rx_data);
  // can_parser();
}

void set_can_enable(bool enable)
{
  can_enable = enable;
}

bool can_is_enable(void)
{
  return can_enable;
}

void can_update(void)
{
  volatile  uint32_t count = hcan2.Instance->RF0R;
  
  if (!can_enable)
    return;

  if (!count) return;
  do {
    // __disable_irq();
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_msg, rx_data);
    can_parser();
    // __enable_irq();
  } while(count--);
}

int CAN2_Send_Msg(uint8_t *tx_data, uint8_t len)
{
	CAN_TxHeaderTypeDef tx_msg;
  uint32_t TxMailbox = CAN_TX_MAILBOX0;
  uint32_t time = 0;

  if (!can_enable)
    return 0;
	
	tx_msg.StdId = 0x12;
	tx_msg.ExtId = 0x12;
	tx_msg.IDE = 0;
	tx_msg.RTR = 0;
	tx_msg.DLC = len;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) { // wait mailbox free
    time++;
    if (time > 500) {
      return -1;
    }
  }
  // DISABLE_ISRS();
  HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, (uint32_t*)&TxMailbox);
  // ENABLE_ISRS();

	if (HAL_OK != status) {
		return -1;
	}
	return 0;
}

uint8_t headtype(void)
{
  return board_type;
}

uint8_t can_read_boardtype(void)
{
  int count = 0;
  uint8_t data = 0xA0;

  if (!can_enable)
    return 0;

  CAN2_Send_Msg(&data, 1);
  board_type_flag = 0;
  while(hcan2.Instance->RF0R == 0) {
    count++;
    delayMicroseconds(1);
    if (count >= 1000) {
      can_timeout++;
      return board_type;
    }
  }
  can_update();

  if (board_type_flag)
    return board_type;

  return board_type;
}

uint8_t can_read_status(void)
{
  int count = 0;
  uint8_t data = 0xA1;
  if (!can_enable)
    return 0;

  CAN2_Send_Msg(&data, 1);
  while(hcan2.Instance->RF0R == 0) {
    count++;
    if (count >= CAN_TIMEOUT) {
      can_timeout++;
      return status;
    }
  }
  can_update();

    return status;
}

float can_read_temperature(void)
{
  float temp = 0.0;
  // int count = 0;
  // uint8_t data = 0xA2;
  // CAN2_Send_Msg(&data, 1);
  // while(hcan2.Instance->RF0R == 0) {
  //   count++;
  //   if (count >= CAN_TIMEOUT) {
  //     can_timeout++;
  //     return head_temperature;
  //   }
  // }
  // can_update();
  // if (head_temperature_flag)
  //   return head_temperature;
  temp = (float)head_temperature;
  if (auto_fan_switch) {
    if (temp > 40){
      can_set_modelfan_en(1);
    }else {
      can_set_modelfan_en(0);
    }
  }

  return temp;
}

void can_temp_update(void)
{
  // if (board_type == 0x03) {
  //   head_temperature = -10;
  //   return;
  // }
  int count = 0;
  uint8_t data = 0xA2;

  if (!can_enable)
    return;

  CAN2_Send_Msg(&data, 1);
  while(hcan2.Instance->RF0R == 0) {
    count++;
    if (count >= CAN_TIMEOUT) {
      can_timeout++;
    }
  }
  can_update();
}

uint16_t can_read_mpu6500(void)
{
  int count = 0;
  uint8_t data = 0xA3;

  if (!can_enable)
    return 0;

  CAN2_Send_Msg(&data, 1);
  while(hcan2.Instance->RF0R == 0) {
    count++;
    if (count >= CAN_TIMEOUT) {
      can_timeout++;
      return mpu6500[0];
    }
  }
  can_update();
  if (mpu6500_flag)
    return mpu6500[0];
  return 0;
}

uint8_t can_read_zpro(void)
{
  volatile int count = 0;
  uint8_t data = 0xA6;
  z_pro_flag = 0;

  if (!can_enable)
    return 0;

  

  can_update();
  CAN2_Send_Msg(&data, 1);

  while(hcan2.Instance->RF0R == 0) {
    count++;
    if (count >= CAN_TIMEOUT) {
      can_timeout++;
      return z_pro;
    }
  }
  // delay(1);
    // DELAY_US(100);

  can_update();
  return z_pro;
}

uint8_t can_read_zproxxx(void)
{
  int count = 0;
  uint8_t data = 0xA6;
  z_pro_flag = 0;

  if (!can_enable)
    return 0;

  CAN2_Send_Msg(&data, 1);
  while(hcan2.Instance->RF0R == 0) {
    count++;
    if (count >= 1500) {
        can_timeout++;
        return z_pro;
      }
  }

  can_update();
  return z_pro;
}

void can_parser(void)
{
    switch (rx_data[0]) {
    case 0xA0: {  // read board type
      board_type = rx_data[1];
      board_type_flag = 1;
      break;
    }
    case 0xA1: {  // read status (fan on/off, head power on/off)
      status = rx_data[1];
      status_flag = 1;
      break;
    }
    case 0xA2: {  // read head temperature
      // head_temperature = rx_data[1] | (rx_data[2] << 8);
      memcpy(&head_temperature, &rx_data[1], 4);
      head_temperature_flag = 1;
      break;
    }
    case 0xA3: {  // read mpu6500 status
      mpu6500[0] = rx_data[1] | (rx_data[2] << 8);
      mpu6500[1] = rx_data[3] | (rx_data[4] << 8);
      mpu6500[2] = rx_data[5] | (rx_data[6] << 8);
      mpu6500_flag = 1;
      break;
    }
    case 0xA4: {  // read pwm frequency
      break;
    }
    case 0xA5: {  // read pwm duty
      break;
    }
    case 0xA6: {  // get zpro
      z_pro = rx_data[1];
      z_pro_flag = 1;
      break;
    }
    case 0xC0: {  // zpro interrupt
      z_pro_int = rx_data[1];
      z_pro_int_flag = 1;
      break;
    }
  }
}

void can_set_freq(uint16_t freq, uint16_t duty)
{
  uint8_t data[8] = {0};
  data[0] = 0xB0;
  data[1] = freq & 0xff;
  data[2] = (freq >> 8) & 0xff;
  data[3] = duty & 0xff;
  data[4] = (duty >> 8) & 0xff;
  CAN2_Send_Msg(data, 5);
}

void can_set_pwm(uint8_t duty)
{
  uint8_t data[8] = {0};
  data[0] = 0xB1;
  data[1] = duty & 0xff;
  CAN2_Send_Msg(data, 2);
}

void can_set_headpwr_en(uint8_t enable)
{
  static uint8_t status = false;
  if ((!!enable) == status) {
    return;
  }
  status = !!enable;
  uint8_t data[8] = {0};
  data[0] = 0xB2;
  data[1] = enable & 0xff;
  CAN2_Send_Msg(data, 2);
}

void can_set_headfan_en(uint8_t enable)
{
  static uint8_t status = false;
  if ((!!enable) == status) {
    // return;
  }
  // status = !!enable;
  uint8_t data[8] = {0};
  data[0] = 0xB3;
  data[1] = enable & 0xff;
  CAN2_Send_Msg(data, 2);
}

void can_set_modelfan_en(uint8_t enable)
{
  static uint8_t status = false;
  if ((!!enable) == status) {
    // return;
  }
  // status = !!enable;
  uint8_t data[8] = {0};
  data[0] = 0xB4;
  data[1] = enable & 0xff;
  CAN2_Send_Msg(data, 2);
}

void can_set_zpro_en(uint8_t enable)
{
  uint8_t data[8] = {0};
  data[0] = 0xB5;
  data[1] = enable & 0xff;
  CAN2_Send_Msg(data, 2);
}

void can_set_target_temperature(int16_t temp)
{
  uint8_t data[8] = {0};
  data[0] = 0xB6;
  data[1] = temp & 0xff;
  data[2] = (temp >> 8) & 0xff;
  CAN2_Send_Msg(data, 3);
}

/**************************************************************************************/
void misc_pin_init(void)
{
  SET_INPUT_PULLUP(DOOR_STATUS_PIN);  // READ(DOOR_STATUS_PIN);

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // encoder CHA CHB
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/******************************************************************************************/
// encoder 
static uint32_t encoder;
static uint32_t encoder_counter;
static uint8_t encoder_state;
static uint8_t laster_encoder_state;
static uint8_t enable_encoder;

void encoder_update(void)
{
  if (!enable_encoder)
    return;

  encoder_counter++;
  if (encoder_counter < 10)
    return;

  if (encoder_counter >= 10) {
    encoder_counter = 0;
    encoder_state = LL_GPIO_IsInputPinSet(GPIOE, LL_GPIO_PIN_14);
  }
  if (encoder_state != laster_encoder_state) {
    encoder = 0;
    laster_encoder_state = encoder_state;
  } else {
    encoder++;
  }
}

void encoder_enable(void)
{
  encoder = 0;
  encoder_counter = 0;
  encoder_state = 0;
  laster_encoder_state = 0;
  enable_encoder = 1;
}

int encoder_get_statue(void)
{
  if (encoder > 12000)
    return 1;
  else
    return 0;
}

int encoder_get_value(void)
{
  return encoder;
}


#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
