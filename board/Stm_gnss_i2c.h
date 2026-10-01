// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// u-blox receiver on I2C3 (DDC interface). Reads the raw NMEA byte stream.

#include <stm32g0xx_hal.h>
#include <stm32g0xx_hal_gpio.h>
#include <stm32g0xx_hal_i2c.h>

#include <cstdint>
#include <cstring>

// u-blox dynamic platform model: 6 = airborne <1g, 7 = airborne <2g, 8 = airborne <4g
#ifndef GNSS_DYNAMIC_MODEL
#define GNSS_DYNAMIC_MODEL 8
#endif

class Stm_gnss_i2c
{
public:
  // meas_period_ms is the interval between navigation solutions (100 ms is 10 Hz, the fastest
  // the receiver supports)
  explicit Stm_gnss_i2c(uint16_t meas_period_ms)
      : meas_period_ms(meas_period_ms < 100 ? 100 : meas_period_ms)
  {
  }

  // Sets up the I2C peripheral and powers the receiver. Does not wait for the receiver.
  // Returns false if the peripheral itself could not be set up.
  bool init()
  {
    i2c.Instance = I2C3;
    i2c.Init.Timing = 0x00300617;
    i2c.Init.OwnAddress1 = 132;
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
    i2c.Init.OwnAddress2 = 0;
    i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    peripheral_ok = HAL_I2C_Init(&i2c) == HAL_OK &&
                    HAL_I2CEx_ConfigAnalogFilter(&i2c, I2C_ANALOGFILTER_ENABLE) == HAL_OK &&
                    HAL_I2CEx_ConfigDigitalFilter(&i2c, 0) == HAL_OK;

    // Enable the receiver (PB6)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    return peripheral_ok;
  }

  // One attempt to reach the receiver. On success it is configured and true is returned.
  // Call repeatedly until it succeeds.
  bool connect()
  {
    if (!peripheral_ok || HAL_I2C_IsDeviceReady(&i2c, address, 1, 20) != HAL_OK)
    {
      return false;
    }
    configure();
    return true;
  }

  // Sends the receiver configuration. The receiver keeps it in RAM only, so it has to be sent
  // again if the receiver restarts.
  void configure()
  {
    set_dynamic_model();
    set_measurement_period();
  }

  uint16_t period_ms() const
  {
    return meas_period_ms;
  }

  // Reads up to cap bytes of pending data. Returns the number of bytes read.
  uint32_t read(uint8_t *buf, uint32_t cap)
  {
    if (!peripheral_ok)
    {
      return 0;
    }
    // The receiver reports the number of bytes waiting at registers 0xFD (MSB) and 0xFE (LSB)
    uint8_t len_regs[2] = {0, 0};
    if (HAL_I2C_Mem_Read(&i2c, address, 0xFD, I2C_MEMADD_SIZE_8BIT, len_regs, 2, 100) != HAL_OK)
    {
      return 0;
    }
    uint32_t available = ((uint32_t)len_regs[0] << 8) | len_regs[1];
    if (available > cap)
    {
      available = cap;
    }
    if (available == 0)
    {
      return 0;
    }
    // Register 0xFF is the data stream
    if (HAL_I2C_Mem_Read(&i2c, address, 0xFF, I2C_MEMADD_SIZE_8BIT, buf, available, 100) != HAL_OK)
    {
      return 0;
    }
    return available;
  }

private:
  // Sends a UBX-CFG-VALSET to the receiver's RAM layer. items is a list of (key, value) pairs
  // already packed as the receiver expects: 4 byte little endian key, then the value bytes.
  void send_valset(const uint8_t *items, uint8_t items_len)
  {
    uint8_t cmd[40];
    const uint8_t payload_len = 4 + items_len; // version, layer, 2 reserved bytes, then the items
    cmd[0] = 0xB5;
    cmd[1] = 0x62;
    cmd[2] = 0x06; // UBX-CFG-VALSET
    cmd[3] = 0x8A;
    cmd[4] = payload_len;
    cmd[5] = 0x00;
    cmd[6] = 0x00; // version
    cmd[7] = 0x01; // layer: RAM
    cmd[8] = 0x00;
    cmd[9] = 0x00;
    memcpy(&cmd[10], items, items_len);
    uint8_t ck_a = 0, ck_b = 0; // Fletcher checksum over class, id, length and payload
    for (int i = 2; i < 6 + payload_len; i++)
    {
      ck_a += cmd[i];
      ck_b += ck_a;
    }
    cmd[6 + payload_len] = ck_a;
    cmd[7 + payload_len] = ck_b;
    HAL_I2C_Mem_Write(&i2c, address, 0xFF, I2C_MEMADD_SIZE_8BIT, cmd, 8 + payload_len, 1000);
  }

  void set_measurement_period()
  {
    // CFG-RATE-MEAS (0x30210001), U2
    const uint8_t item[6] = {0x01, 0x00, 0x21, 0x30, (uint8_t)(meas_period_ms & 0xFF), (uint8_t)(meas_period_ms >> 8)};
    send_valset(item, sizeof(item));
  }

  // The receiver's default dynamic platform model limits speed, altitude and acceleration far below
  // what a UAV can reach. Airborne <4g allows 500 m/s and the highest accelerations.
  void set_dynamic_model()
  {
    // CFG-NAVSPG-DYNMODEL (0x20110021), E1
    const uint8_t item[5] = {0x21, 0x00, 0x11, 0x20, GNSS_DYNAMIC_MODEL};
    send_valset(item, sizeof(item));
  }

#ifndef GNSS_I2C_ADDRESS
#define GNSS_I2C_ADDRESS 0x84 // 7-bit address 0x42, shifted for the HAL
#endif
  static constexpr uint8_t address = GNSS_I2C_ADDRESS;
  uint16_t meas_period_ms;
  bool peripheral_ok{false};
  I2C_HandleTypeDef i2c{};
};
