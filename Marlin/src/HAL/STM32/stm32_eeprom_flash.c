/**
  ******************************************************************************
  * @file    stm32_flash.c
  * @brief   Provides emulated flash from flash
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "stm32_eeprom_flash.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADDR_FLASH_SECTOR_2		0x08008000  // 0x0800 8000 - 0x0800 BFFF
#define ADDR_FLASH_SECTOR_3		0x0800C000  // 0x0800 C000 - 0x0800 FFFF

#define FLASH_FACTORY_DATA_SECTOR   ((uint32_t)(FLASH_SECTOR_2))
#define FLASH_USER_DATA_SECTOR      ((uint32_t)(FLASH_SECTOR_3))

/* Be able to change FLASH_BASE_ADDRESS to use */
#define FLASH_FACTORY_BASE_ADDRESS  (uint32_t)(ADDR_FLASH_SECTOR_2)
#define FLASH_USER_BASE_ADDRESS     (uint32_t)(ADDR_FLASH_SECTOR_3)

static uint32_t flash_base_address = FLASH_USER_BASE_ADDRESS;
static uint32_t flash_sector = FLASH_USER_DATA_SECTOR;
static uint8_t flash_buffer[E2END + 1] = {0};

void change_to_factory_address(void)
{
  flash_sector = FLASH_FACTORY_DATA_SECTOR;
  flash_base_address = FLASH_FACTORY_BASE_ADDRESS;
}

void change_to_user_address(void)
{
  flash_sector = FLASH_USER_DATA_SECTOR;
  flash_base_address = FLASH_USER_BASE_ADDRESS;
}

/**
  * @brief  Function reads a byte from emulated flash (flash)
  * @param  pos : address to read
  * @retval byte : data read from flash
  */
uint8_t flash_read_byte(const uint32_t pos)
{
  flash_buffer_fill();
  return flash_buffered_read_byte(pos);
}

/**
  * @brief  Function writes a byte to emulated flash (flash)
  * @param  pos : address to write
  * @param  value : value to write
  * @retval none
  */
void flash_write_byte(uint32_t pos, uint8_t value)
{
  flash_buffered_write_byte(pos, value);
  flash_buffer_flush();
}

/**
  * @brief  Function reads a byte from the flash buffer
  * @param  pos : address to read
  * @retval byte : data read from flash
  */
uint8_t flash_buffered_read_byte(const uint32_t pos)
{
  return flash_buffer[pos];
}

/**
  * @brief  Function writes a byte to the flash buffer
  * @param  pos : address to write
  * @param  value : value to write
  * @retval none
  */
void flash_buffered_write_byte(uint32_t pos, uint8_t value)
{
  flash_buffer[pos] = value;
}

/**
  * @brief  This function copies the data from flash into the buffer
  * @param  none
  * @retval none
  */
void flash_buffer_fill(void)
{
  memcpy(flash_buffer, (uint8_t *)(flash_base_address), E2END + 1);
}

/**
  * @brief  This function writes the buffer content into the flash
  * @param  none
  * @retval none
  */
int flash_buffer_flush(void)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t offset = 0;
  int ret = 0;
  uint32_t address = flash_base_address;
  uint32_t address_end = flash_base_address + E2END;
  uint32_t SectorError = 0;
  uint32_t data = 0;

  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = flash_sector;
  EraseInitStruct.NbSectors = 1;

  HAL_FLASH_Unlock();

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
    while (address <= address_end) {
      memcpy(&data, flash_buffer + offset, sizeof(uint32_t));
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK) {
        address += 4;
        offset += 4;

      } else {
        address = address_end + 1;
      }
    }
  } else {
    ret = -1;
  }
  
  HAL_FLASH_Lock();
  return ret;
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
