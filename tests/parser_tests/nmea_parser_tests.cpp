// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#include <Gnss_canard_messages.h>
#include <Nmea_parser.h>

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

namespace
{
// Wraps a sentence body in $...*hh with a correct checksum
std::string nmea(const std::string &body)
{
  uint8_t sum = 0;
  for (char c : body)
  {
    sum ^= (uint8_t)c;
  }
  char tail[8];
  ::snprintf(tail, sizeof(tail), "*%02X\r\n", (unsigned)sum);
  return "$" + body + tail;
}

void feed(Nmea_parser &p, const std::string &s)
{
  p.feed(reinterpret_cast<const uint8_t *>(s.data()), s.size());
}

std::string full_fix_epoch()
{
  return nmea("GNGGA,123519.00,4807.03800,N,01131.00000,E,1,12,0.9,545.4,M,47.0,M,,") +
         nmea("GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.6,0.9,1.3,1") +
         nmea("GPGSV,2,1,08,01,40,083,46,02,17,308,41,03,12,113,37,04,22,200,40,1") +
         nmea("GPGSV,2,2,08,05,40,083,46,06,17,308,41,07,12,113,37,08,22,200,40,1") +
         nmea("GAGSV,1,1,03,11,40,083,46,12,17,308,41,13,12,113,37,7") +
         nmea("GNRMC,123519.00,A,4807.03800,N,01131.00000,E,10.0,90.0,230326,,,A,V");
}
} // namespace

TEST(Nmea_parser, full_fix)
{
  Nmea_parser p;
  feed(p, full_fix_epoch());
  ASSERT_TRUE(p.take_epoch());
  EXPECT_FALSE(p.take_epoch());

  const Gnss_state &s = p.state();
  EXPECT_TRUE(s.have_position);
  EXPECT_NEAR(s.latitude_deg, 48.117300, 1e-6);
  EXPECT_NEAR(s.longitude_deg, 11.516667, 1e-6);
  EXPECT_NEAR(s.altitude_msl_m, 545.4, 1e-9);
  EXPECT_NEAR(s.geoid_separation_m, 47.0, 1e-9);
  EXPECT_EQ(s.fix_quality, 1);
  EXPECT_EQ(s.fix_type, 3);
  EXPECT_EQ(s.sats_used, 12);
  EXPECT_EQ(s.sats_visible, 11); // 8 GPS + 3 Galileo
  EXPECT_NEAR(s.pdop, 1.6, 1e-5);
  EXPECT_NEAR(s.hdop, 0.9, 1e-5);
  EXPECT_NEAR(s.vdop, 1.3, 1e-5);
  EXPECT_NEAR(s.speed_mps, 5.14444, 1e-4);
  EXPECT_NEAR(s.course_deg, 90.0, 1e-5);
  // 2026-03-23 12:35:19 UTC
  EXPECT_EQ(s.utc_usec, 1774269319ULL * 1000000ULL);
}

TEST(Nmea_parser, southern_western_hemisphere_is_negative)
{
  Nmea_parser p;
  feed(p, nmea("GNGGA,000000.00,3351.00000,S,15112.00000,W,1,08,1.0,10.0,M,20.0,M,,") +
              nmea("GNRMC,000000.00,A,3351.00000,S,15112.00000,W,0.0,0.0,010124,,,A,V"));
  ASSERT_TRUE(p.take_epoch());
  EXPECT_NEAR(p.state().latitude_deg, -33.85, 1e-6);
  EXPECT_NEAR(p.state().longitude_deg, -151.2, 1e-6);
}

// Sentences captured from the receiver with no satellites in view. The empty fields must not
// shift the later ones.
TEST(Nmea_parser, no_fix_with_empty_fields)
{
  Nmea_parser p;
  feed(p, "$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n"
          "$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,3*31\r\n"
          "$GAGSV,1,1,00,0*74\r\n"
          "$GNRMC,,V,,,,,,,,,,N,V*37\r\n"
          "$GNVTG,,,,,,,,,N*2E\r\n");
  ASSERT_TRUE(p.take_epoch());
  const Gnss_state &s = p.state();
  EXPECT_FALSE(s.have_position);
  EXPECT_EQ(s.fix_quality, 0);
  EXPECT_EQ(s.fix_type, 1);
  EXPECT_EQ(s.sats_visible, 0);
  EXPECT_EQ(s.utc_usec, 0u);
  EXPECT_FALSE(s.have_velocity);
  EXPECT_NEAR(s.hdop, 99.99, 1e-4);
}

TEST(Nmea_parser, rejects_bad_checksum)
{
  Nmea_parser p;
  std::string s = nmea("GNRMC,123519.00,A,4807.03800,N,01131.00000,E,10.0,90.0,230326,,,A,V");
  s[s.size() - 4] = s[s.size() - 4] == '0' ? '1' : '0'; // corrupt a checksum digit
  feed(p, s);
  EXPECT_FALSE(p.take_epoch());
}

TEST(Nmea_parser, handles_data_split_across_reads)
{
  Nmea_parser p;
  const std::string all = full_fix_epoch();
  for (size_t i = 0; i < all.size(); i += 7)
  {
    feed(p, all.substr(i, 7));
  }
  EXPECT_TRUE(p.take_epoch());
  EXPECT_TRUE(p.state().have_position);
}

TEST(Nmea_parser, ignores_garbage_and_overlong_lines)
{
  Nmea_parser p;
  feed(p, std::string(300, 'x') + "\r\n$" + std::string(300, 'A') + "\r\n");
  feed(p, std::string("\xff\xff\xff", 3));
  feed(p, full_fix_epoch());
  EXPECT_TRUE(p.take_epoch());
}

TEST(Days_from_civil, known_dates)
{
  EXPECT_EQ(Nmea_parser::days_from_civil(1970, 1, 1), 0);
  EXPECT_EQ(Nmea_parser::days_from_civil(2000, 3, 1), 11017);
  EXPECT_EQ(Nmea_parser::days_from_civil(2026, 9, 30), 20726);
}

static uavcan_equipment_gnss_Fix2 fix2_from(const Nmea_parser &p)
{
  uavcan_equipment_gnss_Fix2 msg;
  Gnss_canard::make_fix2(p.state(), msg);
  return msg;
}

TEST(Fix2_message, units_and_fields)
{
  Nmea_parser p;
  feed(p, full_fix_epoch());
  p.take_epoch();
  const auto msg = fix2_from(p);
  EXPECT_EQ(msg.latitude_deg_1e8, 4811730000);
  EXPECT_EQ(msg.longitude_deg_1e8, 1151666667);
  EXPECT_EQ(msg.height_msl_mm, 545400);
  EXPECT_EQ(msg.height_ellipsoid_mm, 592400);
  EXPECT_EQ(msg.status, UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX);
  EXPECT_EQ(msg.sats_used, 12);
  EXPECT_EQ(msg.gnss_time_standard, UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC);
  EXPECT_EQ(msg.gnss_timestamp.usec, 1774269319ULL * 1000000ULL);
  EXPECT_NEAR(msg.ned_velocity[0], 0.0f, 1e-4);   // heading east
  EXPECT_NEAR(msg.ned_velocity[1], 5.14444f, 1e-3);

  uavcan_equipment_gnss_Auxiliary aux;
  Gnss_canard::make_aux(p.state(), aux);
  EXPECT_EQ(aux.sats_visible, 11);
  EXPECT_EQ(aux.sats_used, 12);
  EXPECT_NEAR(aux.hdop, 0.9f, 1e-5);
}

TEST(Fix2_message, no_fix)
{
  Nmea_parser p;
  feed(p, "$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n$GNRMC,,V,,,,,,,,,,N,V*37\r\n");
  const auto msg = fix2_from(p);
  EXPECT_EQ(msg.status, UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX);
  EXPECT_EQ(msg.gnss_time_standard, UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_NONE);
  EXPECT_EQ(msg.gnss_timestamp.usec, 0u);
  EXPECT_EQ(msg.latitude_deg_1e8, 0);
}

// The generated encoders must link and produce a payload that fits the message
TEST(Fix2_message, encodes_within_maximum_size)
{
  Nmea_parser p;
  feed(p, full_fix_epoch());
  p.take_epoch();
  auto msg = fix2_from(p);
  uint8_t buf[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE] = {};
  const uint32_t len = uavcan_equipment_gnss_Fix2_encode(&msg, buf);
  EXPECT_GT(len, 40u);
  EXPECT_LE(len, (uint32_t)UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE);
}
