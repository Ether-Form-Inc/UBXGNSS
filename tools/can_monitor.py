#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
"""Minimal DroneCAN GNSS monitor for the serial USB-CAN analyzer (1 Mbit/s CAN, 2 Mbaud serial).

Prints the Fix2 (1063) and Auxiliary (1061) broadcasts of one node and the rate they arrive at.
Usage: can_monitor.py [/dev/ttyUSB0] [node_id] [seconds]
"""
import struct
import sys
import time
import collections
import serial

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
node = int(sys.argv[2]) if len(sys.argv) > 2 else 15
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 10

# The adapter only streams while DTR and RTS are asserted, so set them before the port opens.
# With pyserial's defaults the stream stops after about a second.
s = serial.Serial(None, 2000000, timeout=0.05, stopbits=serial.STOPBITS_TWO)
s.port = dev
s.dtr = True
s.rts = True
s.open()
cmd = bytearray([0xAA, 0x55, 0x12, 0x01, 0x01] + [0] * 8 + [0x00, 0x01, 0, 0, 0, 0])  # 1 Mbit/s, normal
cmd.append(sum(cmd[2:]) & 0xFF)
s.write(cmd)
s.reset_input_buffer()


consumed = 0


def frames(buf):
    """Yields (can_id, data) for each complete frame; sets `consumed` to the bytes fully handled."""
    global consumed
    i = 0
    consumed = 0
    while i < len(buf):
        if buf[i] == 0xAA and i + 1 < len(buf) and (buf[i + 1] & 0xC0) == 0xC0:
            dlc = buf[i + 1] & 0x0F
            end = i + 6 + dlc
            if end >= len(buf):
                break  # incomplete frame: keep it for the next read
            if buf[end] == 0x55:
                yield int.from_bytes(buf[i + 2:i + 6], "little"), bytes(buf[i + 6:end])
                i = end + 1
                consumed = i
                continue
        i += 1
        consumed = i


def bits(data, off, n, signed=False):
    v = int.from_bytes(data, "little") >> off & ((1 << n) - 1)
    if signed and v >> (n - 1):
        v -= 1 << n
    return v


def half(v):
    return struct.unpack("<e", struct.pack("<H", v))[0]


def decode_fix2(p):
    # Field layout from uavcan.equipment.gnss.Fix2 (1063)
    o = 0
    out = {}
    o += 56  # timestamp
    out["utc_us"] = bits(p, o, 56); o += 56
    o += 3 + 13 + 8  # time standard, void13, leap seconds
    out["lon"] = bits(p, o, 37, True) / 1e8; o += 37
    out["lat"] = bits(p, o, 37, True) / 1e8; o += 37
    out["h_ell_m"] = bits(p, o, 27, True) / 1e3; o += 27
    out["h_msl_m"] = bits(p, o, 27, True) / 1e3; o += 27
    out["ned"] = struct.unpack("<3f", p_bits_bytes(p, o, 96)); o += 96
    out["sats"] = bits(p, o, 6); o += 6
    out["status"] = bits(p, o, 2); o += 2
    out["mode"] = bits(p, o, 4); o += 4
    o += 6  # sub mode
    return out


def p_bits_bytes(p, off, n):
    v = int.from_bytes(p, "little") >> off & ((1 << n) - 1)
    return v.to_bytes(n // 8, "little")


def decode_aux(p):
    d = [half(bits(p, 16 * k, 16)) for k in range(7)]
    return {"gdop": d[0], "pdop": d[1], "hdop": d[2], "vdop": d[3], "visible": bits(p, 112, 7), "used": bits(p, 119, 6)}


SIGNATURES = {1063: 0xCA41E7000F37435F, 1061: 0x9BE8BDC4C3DBBFD2}


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


crc_bad = collections.Counter()
last_node_status = None
transfers = {}
count = collections.Counter()
times = collections.defaultdict(list)
t0 = time.time()
buf = bytearray()
while time.time() - t0 < duration:
    buf += s.read(4096)
    consumed = 0
    for cid, d in frames(buf):
        typ, src = (cid >> 8) & 0xFFFF, cid & 0x7F
        if cid & 0x80 or src != node or not d:
            continue
        tail = d[-1]
        sot, eot, tog, tid = tail >> 7, tail >> 6 & 1, tail >> 5 & 1, tail & 31
        key = (typ, src)
        if sot:
            transfers[key] = bytearray()
        if key not in transfers:
            continue
        transfers[key] += d[:-1]
        if eot:
            p = bytes(transfers.pop(key))
            if typ in (1063, 1061):
                # Multi-frame transfers start with a CRC-16 over the data type signature and payload
                sig = SIGNATURES[typ].to_bytes(8, "little")
                if crc16(sig + p[2:]) != int.from_bytes(p[:2], "little"):
                    crc_bad[typ] += 1
                p = p[2:]
            count[typ] += 1
            if typ == 341 and len(p) >= 7:
                # NodeStatus: uptime 32, health 2, mode 3, sub mode 3, vendor status 16 (little endian)
                # DroneCAN packs sub-byte fields from the top of the byte: health in bits 7..6, mode in 5..3
                last_node_status = (bits(p, 0, 32), p[4] >> 6, (p[4] >> 3) & 7, bits(p, 40, 16))
            times[typ].append(time.time())
            if typ == 1063 and count[typ] <= 3:
                print("Fix2", decode_fix2(p))
            if typ == 1061 and count[typ] <= 2:
                print("Aux ", decode_aux(p))
    del buf[:consumed]

for typ, n in sorted(count.items()):
    t = times[typ]
    rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0
    print(f"type {typ}: {n} transfers, {rate:.2f} Hz, CRC errors: {crc_bad[typ]}")
if last_node_status:
    names = ["OK", "WARNING", "ERROR", "CRITICAL"]
    print(f"NodeStatus: uptime {last_node_status[0]} s, health {names[last_node_status[1]]}, mode {last_node_status[2]}, vendor status {last_node_status[3]}")
if not count:
    print("no broadcasts received from node", node)
