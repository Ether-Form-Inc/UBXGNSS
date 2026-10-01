// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// Platform independent NMEA 0183 parser. Feed it the raw byte stream from the receiver (in
// chunks of any size) and it keeps the latest GNSS solution in a Gnss_state.
//
// Sentences used:
//   GGA  position, fix quality, satellites used, HDOP, altitude above MSL, geoid separation
//   GSA  fix type (none / 2D / 3D), PDOP, HDOP, VDOP
//   GSV  satellites in view (summed over all constellations)
//   RMC  date and time, ground speed and course. The u-blox receiver outputs RMC last in each
//        epoch, so a parsed RMC marks the end of a navigation epoch (see take_epoch()).

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

struct Gnss_state
{
  static constexpr float nan_f = std::numeric_limits<float>::quiet_NaN();

  // GGA
  uint8_t fix_quality{0}; // 0 = invalid, 1 = GPS, 2 = DGPS, 4 = RTK fixed, 5 = RTK float, ...
  uint8_t sats_used{0};
  bool have_position{false};
  double latitude_deg{0};  // positive north
  double longitude_deg{0}; // positive east
  double altitude_msl_m{0};
  double geoid_separation_m{0}; // ellipsoid height = MSL height + separation

  // GSA
  uint8_t fix_type{1}; // 1 = no fix, 2 = 2D, 3 = 3D
  float pdop{nan_f};
  float hdop{nan_f};
  float vdop{nan_f};

  // GSV
  uint8_t sats_visible{0};

  // RMC
  bool have_velocity{false};
  float speed_mps{0};
  float course_deg{0};

  // Microseconds since 1970-01-01 UTC, 0 if date or time is not known yet
  uint64_t utc_usec{0};
};

class Nmea_parser
{
public:
  // Consume raw bytes from the receiver
  void feed(const uint8_t *data, uint32_t len)
  {
    for (uint32_t i = 0; i < len; i++)
    {
      const char c = (char)data[i];
      if (c == '$')
      {
        line_len = 0;
        in_sentence = true;
      }
      else if (c == '\r' || c == '\n')
      {
        if (in_sentence)
        {
          line[line_len] = '\0';
          process_line();
        }
        in_sentence = false;
        line_len = 0;
      }
      else if (in_sentence)
      {
        if (line_len < sizeof(line) - 1)
        {
          line[line_len++] = c;
        }
        else
        {
          // Too long to be a valid NMEA sentence
          in_sentence = false;
        }
      }
    }
  }

  // Returns true once per completed navigation epoch
  bool take_epoch()
  {
    const bool e = epoch_done;
    epoch_done = false;
    return e;
  }

  const Gnss_state &state() const
  {
    return gnss;
  }

  // Parses ddmm.mmmm / dddmm.mmmm into decimal degrees. Empty string gives 0.
  static double coord_to_decimal(const char *coord, char dir)
  {
    if (coord == nullptr || *coord == '\0')
    {
      return 0.0;
    }
    const double raw = atof(coord);
    const int degrees = (int)(raw / 100.0);
    const double minutes = raw - (double)degrees * 100.0;
    double decimal = degrees + minutes / 60.0;
    if (dir == 'S' || dir == 'W')
    {
      decimal = -decimal;
    }
    return decimal;
  }

  // Days since 1970-01-01 for a proleptic Gregorian date
  static int64_t days_from_civil(int y, unsigned m, unsigned d)
  {
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int64_t)doe - 719468;
  }

private:
  static constexpr int max_fields = 24;
  static constexpr double knots_to_mps = 0.514444;

  // Returns the number of fields. Empty fields are kept. Field 0 is the sentence name.
  int split(char *s, char **fields)
  {
    int n = 0;
    fields[n++] = s;
    for (; *s != '\0' && n < max_fields; s++)
    {
      if (*s == ',')
      {
        *s = '\0';
        fields[n++] = s + 1;
      }
    }
    return n;
  }

  // Checks the optional "*hh" checksum and strips it. Sentences without a checksum are rejected.
  bool check_and_strip_checksum()
  {
    char *star = strrchr(line, '*');
    if (star == nullptr || star[1] == '\0' || star[2] == '\0')
    {
      return false;
    }
    uint8_t sum = 0;
    for (const char *p = line; p < star; p++)
    {
      sum ^= (uint8_t)*p;
    }
    char *end;
    const long expected = strtol(star + 1, &end, 16);
    *star = '\0';
    return end != star + 1 && expected == sum;
  }

  static int talker_index(const char *name)
  {
    // GP = GPS, GL = GLONASS, GA = Galileo, GB/BD = BeiDou, GQ = QZSS
    if (name[0] != 'G' && name[0] != 'B')
    {
      return 5;
    }
    switch (name[1])
    {
    case 'P':
      return 0;
    case 'L':
      return 1;
    case 'A':
      return 2;
    case 'B':
    case 'D':
      return 3;
    case 'Q':
      return 4;
    default:
      return 5;
    }
  }

  static float to_float(const char *s, float fallback)
  {
    return (s == nullptr || *s == '\0') ? fallback : (float)atof(s);
  }

  // hhmmss.ss to seconds since midnight, negative if not valid
  static double time_to_seconds(const char *t)
  {
    if (strlen(t) < 6)
    {
      return -1.0;
    }
    for (int i = 0; i < 6; i++)
    {
      if (t[i] < '0' || t[i] > '9')
      {
        return -1.0;
      }
    }
    const int hh = (t[0] - '0') * 10 + (t[1] - '0');
    const int mm = (t[2] - '0') * 10 + (t[3] - '0');
    const double ss = (t[4] - '0') * 10 + (t[5] - '0') + atof(t + 6);
    return hh * 3600.0 + mm * 60.0 + ss;
  }

  void process_line()
  {
    if (!check_and_strip_checksum())
    {
      return;
    }

    char *f[max_fields];
    const int n = split(line, f);
    const char *name = f[0];
    if (strlen(name) != 5)
    {
      return;
    }
    const char *type = name + 2;

    if (strcmp(type, "GGA") == 0 && n >= 12)
    {
      parse_gga(f);
    }
    else if (strcmp(type, "GSA") == 0 && n >= 18)
    {
      parse_gsa(f);
    }
    else if (strcmp(type, "GSV") == 0 && n >= 4)
    {
      const int idx = talker_index(name);
      const int in_view = atoi(f[3]);
      // The count is repeated in every message of a group, so store it instead of adding
      gsv_in_view[idx] = (uint8_t)(in_view > 255 ? 255 : in_view);
    }
    else if (strcmp(type, "RMC") == 0 && n >= 10)
    {
      parse_rmc(f);
    }
  }

  void parse_gga(char **f)
  {
    gnss.fix_quality = (uint8_t)atoi(f[6]);
    gnss.sats_used = (uint8_t)atoi(f[7]);
    gnss.have_position = gnss.fix_quality > 0 && f[2][0] != '\0' && f[4][0] != '\0';
    if (gnss.have_position)
    {
      gnss.latitude_deg = coord_to_decimal(f[2], f[3][0]);
      gnss.longitude_deg = coord_to_decimal(f[4], f[5][0]);
      gnss.altitude_msl_m = atof(f[9]);
      gnss.geoid_separation_m = atof(f[11]);
    }
    if (f[8][0] != '\0')
    {
      gnss.hdop = (float)atof(f[8]);
    }
  }

  void parse_gsa(char **f)
  {
    const int type = atoi(f[2]);
    gnss.fix_type = (type >= 1 && type <= 3) ? (uint8_t)type : 1;
    gnss.pdop = to_float(f[15], Gnss_state::nan_f);
    gnss.hdop = to_float(f[16], gnss.hdop);
    gnss.vdop = to_float(f[17], Gnss_state::nan_f);
  }

  void parse_rmc(char **f)
  {
    // f[1] time, f[2] status (A = valid), f[7] speed over ground (knots), f[8] course, f[9] date
    const bool valid = f[2][0] == 'A';
    gnss.have_velocity = valid && f[7][0] != '\0';
    gnss.speed_mps = gnss.have_velocity ? (float)(atof(f[7]) * knots_to_mps) : 0.0f;
    gnss.course_deg = to_float(f[8], 0.0f);

    gnss.utc_usec = 0;
    const double tod = time_to_seconds(f[1]);
    const char *date = f[9];
    if (tod >= 0.0 && strlen(date) == 6)
    {
      const int day = (date[0] - '0') * 10 + (date[1] - '0');
      const int month = (date[2] - '0') * 10 + (date[3] - '0');
      const int year = 2000 + (date[4] - '0') * 10 + (date[5] - '0');
      if (day >= 1 && day <= 31 && month >= 1 && month <= 12)
      {
        const int64_t days = days_from_civil(year, (unsigned)month, (unsigned)day);
        gnss.utc_usec = (uint64_t)days * 86400000000ULL + (uint64_t)llround(tod * 1e6);
      }
    }

    // GSV precedes RMC in each epoch, so the counts are complete here
    unsigned visible = 0;
    for (int i = 0; i < 6; i++)
    {
      visible += gsv_in_view[i];
      gsv_in_view[i] = 0;
    }
    gnss.sats_visible = (uint8_t)(visible > 255 ? 255 : visible);

    epoch_done = true;
  }

  Gnss_state gnss{};
  uint8_t gsv_in_view[6]{};
  char line[96]{};
  uint32_t line_len{0};
  bool in_sentence{false};
  bool epoch_done{false};
};
