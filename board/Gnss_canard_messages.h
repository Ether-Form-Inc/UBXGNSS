// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// Builds the C (libcanard / dronecan_dsdlc) GNSS messages from the parsed receiver state.

#include <Nmea_parser.h>

extern "C"
{
#include <uavcan.equipment.gnss.Auxiliary.h>
#include <uavcan.equipment.gnss.Fix2.h>
}

#include <cmath>
#include <cstring>

namespace Gnss_canard
{
inline void make_fix2(const Gnss_state &s, uavcan_equipment_gnss_Fix2 &msg)
{
  memset(&msg, 0, sizeof(msg));

  // Network synchronised time is not available, so timestamp stays zero (unknown)
  msg.gnss_timestamp.usec = s.utc_usec;
  msg.gnss_time_standard = s.utc_usec != 0 ? UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC
                                           : UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_NONE;
  msg.num_leap_seconds = UAVCAN_EQUIPMENT_GNSS_FIX2_NUM_LEAP_SECONDS_UNKNOWN;

  if (s.have_position)
  {
    msg.latitude_deg_1e8 = llround(s.latitude_deg * 1e8);
    msg.longitude_deg_1e8 = llround(s.longitude_deg * 1e8);
    msg.height_msl_mm = lround(s.altitude_msl_m * 1000.0);
    msg.height_ellipsoid_mm = lround((s.altitude_msl_m + s.geoid_separation_m) * 1000.0);
  }

  // Horizontal velocity from ground speed and course. NMEA has no vertical velocity.
  const float course = s.course_deg * (float)(M_PI / 180.0);
  msg.ned_velocity[0] = s.have_velocity ? s.speed_mps * cosf(course) : 0.0f;
  msg.ned_velocity[1] = s.have_velocity ? s.speed_mps * sinf(course) : 0.0f;
  msg.ned_velocity[2] = 0.0f;

  msg.sats_used = s.sats_used;

  if (!s.have_position)
  {
    msg.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
  }
  else if (s.fix_type == 2)
  {
    msg.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX;
  }
  else
  {
    msg.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
  }

  switch (s.fix_quality)
  {
  case 2:
    msg.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS;
    msg.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_SBAS;
    break;
  case 4:
    msg.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
    msg.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
    break;
  case 5:
    msg.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
    msg.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT;
    break;
  default:
    msg.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
    break;
  }

  // Covariance is left empty (unknown); receivers derive accuracy from the DOP in Auxiliary
  msg.covariance.len = 0;
  msg.pdop = s.pdop;
  msg.ecef_position_velocity.len = 0;
}

inline void make_aux(const Gnss_state &s, uavcan_equipment_gnss_Auxiliary &msg)
{
  const float nan = Gnss_state::nan_f;
  msg.gdop = nan;
  msg.pdop = s.pdop;
  msg.hdop = s.hdop;
  msg.vdop = s.vdop;
  msg.tdop = nan;
  msg.ndop = nan;
  msg.edop = nan;
  msg.sats_visible = s.sats_visible;
  msg.sats_used = s.sats_used;
}
} // namespace Gnss_canard
