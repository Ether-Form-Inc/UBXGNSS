// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// DroneCAN GNSS node built on libcanard. Publishes Fix2 and Auxiliary, NodeStatus at 1 Hz, and
// answers GetNodeInfo. All state is static; nothing is allocated outside the libcanard pool.

#include <Gnss_canard_messages.h>
#include <Nmea_parser.h>
#include <Stm_fdcan.h>
#include <Stm_gnss_i2c.h>
#include <Stm_settings.h>

extern "C"
{
#include <canard.h>
#include <uavcan.equipment.gnss.Auxiliary.h>
#include <uavcan.equipment.gnss.Fix2.h>
#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.RestartNode.h>
#include <uavcan.protocol.param.ExecuteOpcode.h>
#include <uavcan.protocol.param.GetSet.h>
}

#include <cstring>

class Canard_gnss_node
{
public:
  Canard_gnss_node(uint8_t node_id, uint16_t meas_period_ms)
      : node_id(node_id), gnss(meas_period_ms)
  {
  }

  // Never returns
  void run()
  {
    fdcan.init();
    canardInit(&canard, pool, sizeof(pool), on_transfer_received, should_accept_transfer, this);
    canardSetLocalNodeID(&canard, node_id);
    gnss.init(); // does not wait for the receiver; service_gnss() connects to it

    for (;;)
    {
      const uint32_t now_ms = HAL_GetTick();
      service_bus_state(now_ms);
      service_restart(now_ms);
      service_rx(now_ms);
      service_gnss(now_ms);
      service_aux(now_ms);
      service_node_status(now_ms);
      service_tx();
      HAL_Delay(1);
    }
  }

private:
  static constexpr uint32_t gnss_poll_period_ms = 10;
  static constexpr uint32_t aux_period_ms = 1000;
  static constexpr uint32_t aux_delay_ms = 30;
  static constexpr uint32_t node_status_period_ms = 1000;
  static constexpr uint32_t cleanup_period_ms = 1000;
  static constexpr uint32_t bus_off_retry_ms = 100;
  static constexpr uint32_t restart_timeout_ms = 500;
  static constexpr int max_gnss_reads_per_poll = 8;
  static constexpr uint32_t receiver_retry_ms = 500;
  static constexpr uint32_t gnss_stale_ms = 3000;

  // ---- receive ----
  void service_rx(uint32_t now_ms)
  {
    uint32_t id;
    uint8_t len;
    CanardCANFrame frame;
    // Bounded so a busy bus cannot starve everything else
    for (int i = 0; i < 16; i++)
    {
      memset(&frame, 0, sizeof(frame));
      if (!fdcan.receive(id, frame.data, len))
      {
        break;
      }
      frame.id = id | CANARD_CAN_FRAME_EFF;
      frame.data_len = len;
      canardHandleRxFrame(&canard, &frame, (uint64_t)now_ms * 1000ULL);
    }

    if ((uint32_t)(now_ms - last_cleanup_ms) >= cleanup_period_ms)
    {
      last_cleanup_ms = now_ms;
      canardCleanupStaleTransfers(&canard, (uint64_t)now_ms * 1000ULL);
    }
  }

  static bool should_accept_transfer(const CanardInstance *, uint64_t *out_signature, uint16_t data_type_id,
                                     CanardTransferType transfer_type, uint8_t)
  {
    if (transfer_type != CanardTransferTypeRequest)
    {
      return false;
    }
    switch (data_type_id)
    {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      *out_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      *out_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
      *out_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      *out_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
      return true;
    default:
      return false;
    }
  }

  static void on_transfer_received(CanardInstance *ins, CanardRxTransfer *transfer)
  {
    Canard_gnss_node *self = static_cast<Canard_gnss_node *>(canardGetUserReference(ins));
    if (transfer->transfer_type != CanardTransferTypeRequest)
    {
      return;
    }
    switch (transfer->data_type_id)
    {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      self->send_node_info(transfer);
      break;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      self->handle_param_getset(transfer);
      break;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
      self->handle_param_execute_opcode(transfer);
      break;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      self->handle_restart_node(transfer);
      break;
    default:
      break;
    }
  }

  // ---- parameters ----
  //
  // One parameter, NODE_ID, so the node ID can be set over CAN (for example from the DroneCAN GUI
  // tool). It is stored in flash and used from the next start. 0 means "not set": the ID jumpers
  // decide. The node ID in use does not change until the node restarts.
  static constexpr const char *node_id_param_name = "NODE_ID";
  static constexpr int64_t node_id_param_max = 127;

  static bool name_is_node_id(const uint8_t *name, uint8_t len)
  {
    const uint8_t expected_len = (uint8_t)strlen(node_id_param_name);
    return len == expected_len && memcmp(name, node_id_param_name, len) == 0;
  }

  static int64_t stored_node_id()
  {
    uint8_t v;
    return Stm_settings::read(Stm_settings::node_id, v) ? v : 0;
  }

  void send_service_response(CanardRxTransfer *request, uint64_t signature, uint8_t type_id, const uint8_t *buf,
                             uint32_t len)
  {
    canardRequestOrRespond(&canard, request->source_node_id, signature, type_id, &request->transfer_id,
                           request->priority, CanardResponse, buf, (uint16_t)len);
  }

  void handle_param_getset(CanardRxTransfer *request)
  {
    static uavcan_protocol_param_GetSetRequest req;
    static uavcan_protocol_param_GetSetResponse res;
    static uint8_t buf[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];

    memset(&req, 0, sizeof(req));
    const bool decode_failed = uavcan_protocol_param_GetSetRequest_decode(request, &req);
    canardReleaseRxTransferPayload(&canard, request); // give the pool blocks back before replying
    if (decode_failed)
    {
      return;
    }

    // Find the parameter by name if one is given, otherwise by index (there is only index 0)
    const bool by_name = req.name.len > 0;
    const bool found = by_name ? name_is_node_id(req.name.data, req.name.len) : req.index == 0;

    memset(&res, 0, sizeof(res));
    if (found)
    {
      if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE)
      {
        const int64_t wanted = req.value.integer_value;
        if (wanted >= 0 && wanted <= node_id_param_max && wanted != stored_node_id())
        {
          if (wanted == 0)
          {
            Stm_settings::erase(Stm_settings::node_id);
          }
          else
          {
            Stm_settings::write(Stm_settings::node_id, (uint8_t)wanted);
          }
        }
        // An out of range value is ignored; the response shows what is actually stored
      }
      res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
      res.value.integer_value = stored_node_id();
      res.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
      res.default_value.integer_value = 0;
      res.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
      res.max_value.integer_value = node_id_param_max;
      res.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
      res.min_value.integer_value = 0;
      res.name.len = (uint8_t)strlen(node_id_param_name);
      memcpy(res.name.data, node_id_param_name, res.name.len);
    }
    // A parameter that does not exist gets an empty response, which ends a listing by index

    memset(buf, 0, sizeof(buf));
    const uint32_t len = uavcan_protocol_param_GetSetResponse_encode(&res, buf);
    send_service_response(request, UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE, UAVCAN_PROTOCOL_PARAM_GETSET_ID, buf, len);
  }

  void handle_param_execute_opcode(CanardRxTransfer *request)
  {
    uavcan_protocol_param_ExecuteOpcodeRequest req = {};
    const bool decode_failed = uavcan_protocol_param_ExecuteOpcodeRequest_decode(request, &req);
    canardReleaseRxTransferPayload(&canard, request);
    if (decode_failed)
    {
      return;
    }

    uavcan_protocol_param_ExecuteOpcodeResponse res = {};
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
    {
      res.ok = true; // GetSet already stored the value in flash
    }
    else if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
    {
      res.ok = Stm_settings::erase(Stm_settings::node_id); // back to the jumper selected ID
    }

    uint8_t buf[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE] = {};
    const uint32_t len = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&res, buf);
    send_service_response(request, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                          UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID, buf, len);
  }

  void handle_restart_node(CanardRxTransfer *request)
  {
    uavcan_protocol_RestartNodeRequest req = {};
    const bool decode_failed = uavcan_protocol_RestartNodeRequest_decode(request, &req);
    canardReleaseRxTransferPayload(&canard, request);
    if (decode_failed)
    {
      return;
    }

    uavcan_protocol_RestartNodeResponse res = {};
    res.ok = req.magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;
    if (res.ok)
    {
      restart_requested = true; // service_restart() resets once the reply has been sent
      restart_requested_ms = HAL_GetTick();
    }

    uint8_t buf[UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE] = {};
    const uint32_t len = uavcan_protocol_RestartNodeResponse_encode(&res, buf);
    send_service_response(request, UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE, UAVCAN_PROTOCOL_RESTARTNODE_ID, buf, len);
  }

  // Resets the MCU after a RestartNode request, once the reply has left the CAN controller
  void service_restart(uint32_t now_ms)
  {
    if (!restart_requested)
    {
      return;
    }
    const bool sent = canardPeekTxQueue(&canard) == nullptr && fdcan.tx_idle();
    if (sent || (uint32_t)(now_ms - restart_requested_ms) > restart_timeout_ms)
    {
      HAL_Delay(5); // let the last frame finish on the bus
      NVIC_SystemReset();
    }
  }

  // Vendor specific status codes in NodeStatus
  static constexpr uint16_t status_ok = 0;
  static constexpr uint16_t status_receiver_not_found = 1; // the GNSS receiver never answered on I2C
  static constexpr uint16_t status_no_data = 2;            // the receiver has stopped producing solutions

  void fill_node_status(uavcan_protocol_NodeStatus &status)
  {
    const uint32_t now_ms = HAL_GetTick();
    status.uptime_sec = now_ms / 1000;
    status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    status.sub_mode = 0;

    // Having no fix is not a fault. A receiver that is missing, or silent, is.
    if (!receiver_found)
    {
      status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;
      status.vendor_specific_status_code = status_receiver_not_found;
    }
    else if ((uint32_t)(now_ms - (have_epoch ? last_epoch_ms : receiver_found_ms)) > gnss_stale_ms)
    {
      status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING;
      status.vendor_specific_status_code = status_no_data;
    }
    else
    {
      status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
      status.vendor_specific_status_code = status_ok;
    }
  }

  void send_node_info(CanardRxTransfer *request)
  {
    static uavcan_protocol_GetNodeInfoResponse res;
    static uint8_t buf[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    memset(&res, 0, sizeof(res));
    memset(buf, 0, sizeof(buf));

    fill_node_status(res.status);
    res.software_version.major = 0;
    res.software_version.minor = 1;
    res.hardware_version.major = 0;

    // 96 bit STM32 unique ID, zero padded to the 128 bits DroneCAN wants
    const uint32_t *uid = reinterpret_cast<const uint32_t *>(0x1FFF7590UL);
    memcpy(res.hardware_version.unique_id, uid, 12);

    static const char name[] = "CAN to GPS";
    res.name.len = sizeof(name) - 1;
    memcpy(res.name.data, name, res.name.len);

    const uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&res, buf
#if CANARD_ENABLE_TAO_OPTION
                                                                   , false
#endif
    );
    canardRequestOrRespond(&canard, request->source_node_id, UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID, &request->transfer_id, request->priority,
                           CanardResponse, buf, (uint16_t)len);
  }

  // ---- transmit ----

  // Recovers from bus-off. Frames queued while the controller cannot transmit are dropped:
  // replaying old GNSS data in a burst once the bus returns would be worse than losing it.
  void service_bus_state(uint32_t now_ms)
  {
    if (!fdcan.bus_off())
    {
      bus_off_seen = false;
      return;
    }

    while (canardPeekTxQueue(&canard) != nullptr)
    {
      canardPopTxQueue(&canard);
    }

    if (!bus_off_seen)
    {
      bus_off_seen = true;
      last_bus_off_action_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - last_bus_off_action_ms) >= bus_off_retry_ms)
    {
      fdcan.restart_after_bus_off();
      last_bus_off_action_ms = now_ms;
    }
  }

  void service_tx()
  {
#ifdef CAN_TX_MIN_SPACING_MS
    // Test aid for slow USB-CAN adapters whose serial link drops back-to-back frames
    static uint32_t last_tx_ms = 0;
    if ((uint32_t)(HAL_GetTick() - last_tx_ms) < CAN_TX_MIN_SPACING_MS)
    {
      return;
    }
#endif

    // The hardware FIFO only holds 3 frames, so feed it as it drains
    while (fdcan.tx_has_room())
    {
      const CanardCANFrame *frame = canardPeekTxQueue(&canard);
      if (frame == nullptr)
      {
        break;
      }
      if (!fdcan.send(frame->id, frame->data, frame->data_len))
      {
        break;
      }
      canardPopTxQueue(&canard);
#ifdef CAN_TX_MIN_SPACING_MS
      last_tx_ms = HAL_GetTick();
      break;
#endif
    }
  }

  void service_node_status(uint32_t now_ms)
  {
    if ((uint32_t)(now_ms - last_node_status_ms) < node_status_period_ms)
    {
      return;
    }
    last_node_status_ms = now_ms;

    uavcan_protocol_NodeStatus status;
    uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    memset(buf, 0, sizeof(buf));
    fill_node_status(status);
    const uint32_t len = uavcan_protocol_NodeStatus_encode(&status, buf
#if CANARD_ENABLE_TAO_OPTION
                                                          , false
#endif
    );
    canardBroadcast(&canard, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &node_status_transfer_id, CANARD_TRANSFER_PRIORITY_LOW, buf, (uint16_t)len);
  }

  // ---- GNSS ----
  void service_gnss(uint32_t now_ms)
  {
    // The receiver may be slow to start, or absent; keep trying without blocking the CAN node
    if (!receiver_found)
    {
      if ((uint32_t)(now_ms - last_connect_try_ms) >= receiver_retry_ms)
      {
        last_connect_try_ms = now_ms;
        if (gnss.connect())
        {
          receiver_found = true;
          receiver_found_ms = now_ms;
          last_reconfig_ms = now_ms;
        }
      }
      return;
    }

    if ((uint32_t)(now_ms - last_gnss_poll_ms) < gnss_poll_period_ms)
    {
      return;
    }
    last_gnss_poll_ms = now_ms;

    // If solutions stop arriving the receiver may have restarted, which loses its setting
    // (held in RAM), so send it again now and then
    const uint32_t quiet_ms = now_ms - (have_epoch ? last_epoch_ms : receiver_found_ms);
    if (quiet_ms > gnss_stale_ms && (uint32_t)(now_ms - last_reconfig_ms) >= receiver_retry_ms)
    {
      last_reconfig_ms = now_ms;
      gnss.configure();
    }

    // Drain what the receiver has buffered (bounded so the CAN node keeps being serviced)
    uint8_t buf[128];
    for (int i = 0; i < max_gnss_reads_per_poll; i++)
    {
      const uint32_t len = gnss.read(buf, sizeof(buf));
      parser.feed(buf, len);
      if (len < sizeof(buf))
      {
        break;
      }
    }

    if (!parser.take_epoch())
    {
      return;
    }
    // Solutions arriving more slowly than configured mean the receiver has lost its setting
    // (for example it restarted and is back at its default 1 Hz), so configure it again
    if (have_epoch && (uint32_t)(now_ms - last_epoch_ms) > (uint32_t)gnss.period_ms() * 3 / 2 &&
        (uint32_t)(now_ms - last_reconfig_ms) >= receiver_retry_ms)
    {
      last_reconfig_ms = now_ms;
      gnss.configure();
    }
    have_epoch = true;
    last_epoch_ms = now_ms;

    static uavcan_equipment_gnss_Fix2 fix2;
    static uint8_t fix2_buf[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE];
    Gnss_canard::make_fix2(parser.state(), fix2);
    memset(fix2_buf, 0, sizeof(fix2_buf));
    const uint32_t fix2_len = uavcan_equipment_gnss_Fix2_encode(&fix2, fix2_buf
#if CANARD_ENABLE_TAO_OPTION
                                                               , false
#endif
    );
    canardBroadcast(&canard, UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE, UAVCAN_EQUIPMENT_GNSS_FIX2_ID,
                    &fix2_transfer_id, CANARD_TRANSFER_PRIORITY_MEDIUM, fix2_buf, (uint16_t)fix2_len);

    // Auxiliary follows Fix2 a little later rather than in the same burst, so the bus never
    // carries more than one multi-frame transfer back to back. Some receivers (including the
    // USB-CAN adapter used for bring-up) buffer only about 8 frames and lose the rest.
    if (!aux_pending && (uint32_t)(now_ms - last_aux_ms) >= aux_period_ms)
    {
      aux_pending = true;
      aux_due_ms = now_ms + aux_delay_ms;
    }
  }

  void service_aux(uint32_t now_ms)
  {
    if (!aux_pending || (int32_t)(now_ms - aux_due_ms) < 0)
    {
      return;
    }
    aux_pending = false;
    last_aux_ms = now_ms;

    uavcan_equipment_gnss_Auxiliary aux;
    uint8_t aux_buf[UAVCAN_EQUIPMENT_GNSS_AUXILIARY_MAX_SIZE];
    memset(aux_buf, 0, sizeof(aux_buf));
    Gnss_canard::make_aux(parser.state(), aux);
    const uint32_t aux_len = uavcan_equipment_gnss_Auxiliary_encode(&aux, aux_buf
#if CANARD_ENABLE_TAO_OPTION
                                                                   , false
#endif
    );
    canardBroadcast(&canard, UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE, UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID,
                    &aux_transfer_id, CANARD_TRANSFER_PRIORITY_LOW, aux_buf, (uint16_t)aux_len);
  }

  uint8_t node_id;
  Stm_fdcan fdcan;
  Stm_gnss_i2c gnss;
  Nmea_parser parser;

  CanardInstance canard;
  alignas(8) uint8_t pool[1024];

  uint8_t node_status_transfer_id{0};
  uint8_t fix2_transfer_id{0};
  uint8_t aux_transfer_id{0};

  uint32_t last_gnss_poll_ms{0};
  uint32_t last_aux_ms{0};
  bool receiver_found{false};
  uint32_t receiver_found_ms{0};
  uint32_t last_connect_try_ms{0};
  uint32_t last_reconfig_ms{0};
  bool have_epoch{false};
  uint32_t last_epoch_ms{0};
  uint32_t aux_due_ms{0};
  bool aux_pending{false};
  uint32_t last_node_status_ms{0};
  uint32_t last_cleanup_ms{0};
  uint32_t last_bus_off_action_ms{0};
  bool bus_off_seen{false};
  bool restart_requested{false};
  uint32_t restart_requested_ms{0};
};
