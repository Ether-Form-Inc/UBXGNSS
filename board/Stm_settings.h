// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// Settings kept in the last pages of flash, which the linker script keeps the application out of.
// Each setting is one byte in its own page. Erased flash (all 0xFF) means "not set".

#include <stm32g0xx_hal.h>

#include <cstdint>

namespace Stm_settings
{
struct Setting
{
  uint32_t page;
  uint32_t address;
};

constexpr Setting node_id{62, 0x0801F000};     // node ID override, 1 to 127
constexpr Setting update_rate{63, 0x0801F800}; // receiver measurement period, ms

// Reads a one byte setting. Returns false if it was never written (or holds 0xFF).
inline bool read(const Setting &s, uint8_t &value)
{
  const uint64_t raw = *(const volatile uint64_t *)s.address;
  value = (uint8_t)raw;
  return raw != 0xFFFFFFFFFFFFFFFFULL && value != 0xFF;
}

// Erases the setting, so it reads as "not set". Returns true on success.
inline bool erase(const Setting &s)
{
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                         FLASH_FLAG_SIZERR | FLASH_FLAG_PGSERR | FLASH_FLAG_MISERR | FLASH_FLAG_FASTERR);

  FLASH_EraseInitTypeDef erase_init = {};
  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.Banks = FLASH_BANK_1;
  erase_init.Page = s.page;
  erase_init.NbPages = 1;
  uint32_t page_error = 0;
  const bool ok = HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK;
  HAL_FLASH_Lock();
  return ok;
}

// Stores a one byte setting (the page is erased first). The CPU is stalled while flash is
// erased, a few tens of milliseconds. Returns true if the value reads back correctly.
inline bool write(const Setting &s, uint8_t value)
{
  if (!erase(s))
  {
    return false;
  }
  HAL_FLASH_Unlock();
  const bool ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, s.address, (uint64_t)value) == HAL_OK;
  HAL_FLASH_Lock();

  uint8_t check;
  return ok && read(s, check) && check == value;
}
} // namespace Stm_settings
