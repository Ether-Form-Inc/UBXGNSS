// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// Minimal classic CAN driver for the STM32G0 FDCAN peripheral: 1 Mbit/s, 29-bit IDs, receive all.
// Has no dependency on a CAN stack; frames are plain id / data / length.

#include <stm32g0xx_hal.h>

#include <cstdint>
#include <cstring>

class Stm_fdcan
{
public:
  bool init()
  {
    hfdcan.Instance = FDCAN1;
    hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan.Init.AutoRetransmission = ENABLE; // a frame lost to arbitration must be retried
    hfdcan.Init.TransmitPause = DISABLE;
    hfdcan.Init.ProtocolException = DISABLE;
    // 16 MHz FDCAN clock, prescaler 4, 4 time quanta per bit: 1 Mbit/s, 75% sample point
    hfdcan.Init.NominalPrescaler = 4;
    hfdcan.Init.NominalSyncJumpWidth = 1;
    hfdcan.Init.NominalTimeSeg1 = 2;
    hfdcan.Init.NominalTimeSeg2 = 1;
    hfdcan.Init.DataPrescaler = 1;
    hfdcan.Init.DataSyncJumpWidth = 1;
    hfdcan.Init.DataTimeSeg1 = 1;
    hfdcan.Init.DataTimeSeg2 = 1;
    hfdcan.Init.StdFiltersNbr = 0;
    hfdcan.Init.ExtFiltersNbr = 0;
    hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    bool ok = HAL_FDCAN_Init(&hfdcan) == HAL_OK;
    ok &= HAL_FDCAN_Start(&hfdcan) == HAL_OK;

    // Accept all extended IDs into RX FIFO 0
    FDCAN_FilterTypeDef filter = {};
    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    ok &= HAL_FDCAN_ConfigFilter(&hfdcan, &filter) == HAL_OK;

    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    return ok;
  }

  // The controller goes bus-off when its transmit error counter passes 255, typically because
  // nothing is acknowledging its frames (no other node, a wiring fault, or a bit rate mismatch).
  // The hardware then sets INIT and stays silent until software clears it.
  bool bus_off() const
  {
    return (hfdcan.Instance->PSR & FDCAN_PSR_BO) != 0;
  }

  // Leaves INIT; the controller waits for 128 occurrences of 11 recessive bits before it
  // takes part in the bus again.
  void restart_after_bus_off()
  {
    CLEAR_BIT(hfdcan.Instance->CCCR, FDCAN_CCCR_INIT);
  }

  // True when everything handed to the controller has been transmitted
  bool tx_idle()
  {
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) == 3;
  }

  // The hardware TX FIFO holds 3 frames
  bool tx_has_room()
  {
    return HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) > 0;
  }

  bool send(uint32_t id, const uint8_t *data, uint8_t len)
  {
    tx_header.Identifier = id & 0x1FFFFFFFU;
    tx_header.DataLength = len; // the HAL uses the raw DLC code, which equals the length up to 8
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &tx_header, data) == HAL_OK;
  }

  // Returns true and fills the outputs if a frame was waiting
  bool receive(uint32_t &id, uint8_t *data, uint8_t &len)
  {
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan, FDCAN_RX_FIFO0) == 0)
    {
      return false;
    }
    FDCAN_RxHeaderTypeDef rx_header;
    if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK)
    {
      return false;
    }
    id = rx_header.Identifier;
    len = rx_header.DataLength > 8 ? 8 : (uint8_t)rx_header.DataLength;
    return rx_header.IdType == FDCAN_EXTENDED_ID && rx_header.RxFrameType == FDCAN_DATA_FRAME;
  }

private:
  FDCAN_HandleTypeDef hfdcan{};
  FDCAN_TxHeaderTypeDef tx_header{};
};
