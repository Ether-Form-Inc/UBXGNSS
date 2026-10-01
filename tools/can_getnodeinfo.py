#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
"""Sends a uavcan.protocol.GetNodeInfo request to a node through the serial USB-CAN adapter and
decodes the response.  Usage: can_getnodeinfo.py [/dev/ttyUSB0] [node_id]"""
import sys
import time
import serial

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
node = int(sys.argv[2]) if len(sys.argv) > 2 else 100
SELF = 50
SIGNATURE = 0xEE468A8121C46A9E

s = serial.Serial(None, 2000000, timeout=0.05, stopbits=serial.STOPBITS_TWO)
s.port = dev
s.dtr = True
s.rts = True
s.open()
cfg = bytearray([0xAA, 0x55, 0x12, 0x01, 0x01] + [0] * 8 + [0x00, 0x01, 0, 0, 0, 0])
cfg.append(sum(cfg[2:]) & 0xFF)
s.write(cfg)
time.sleep(0.3)
s.reset_input_buffer()


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def bits(data, off, n):
    return int.from_bytes(data, "little") >> off & ((1 << n) - 1)


# Service request: priority 16, service type 1, request bit, destination, service flag, source
req_id = (16 << 24) | (1 << 16) | (1 << 15) | (node << 8) | (1 << 7) | SELF
tid = 7
s.write(bytes([0xAA, 0xE0 | 1]) + req_id.to_bytes(4, "little") + bytes([0xC0 | tid]) + b"\x55")

want_id = (1 << 16) | (SELF << 8) | (1 << 7) | node   # response: request bit clear
buf = bytearray()
payload = None
t0 = time.time()
while time.time() - t0 < 3 and payload is None:
    buf += s.read(4096)
    i = 0
    frames = []
    while i < len(buf) - 7:
        if buf[i] == 0xAA and (buf[i + 1] & 0xC0) == 0xC0:
            dlc = buf[i + 1] & 0x0F
            end = i + 6 + dlc
            if end < len(buf) and buf[end] == 0x55:
                frames.append((int.from_bytes(buf[i + 2:i + 6], "little"), bytes(buf[i + 6:end])))
                i = end + 1
                continue
        i += 1
    assembled = bytearray()
    for cid, d in frames:
        if cid & 0x00FFFFFF == want_id & 0x00FFFFFF and d:
            tail = d[-1]
            if tail & 0x80:
                assembled = bytearray()
            assembled += d[:-1]
            if tail & 0x40:
                payload = bytes(assembled)

if payload is None:
    sys.exit("no GetNodeInfo response received")

crc_ok = crc16(SIGNATURE.to_bytes(8, "little") + payload[2:]) == int.from_bytes(payload[:2], "little")
p = payload[2:]
o = 0
uptime = bits(p, o, 32); o += 32
# DroneCAN packs sub-byte fields from the top of the byte
health = p[o // 8] >> 6; o += 2
mode = (p[o // 8 - 0] >> 3) & 7; o += 3
o += 3                      # sub_mode
vendor = bits(p, o, 16); o += 16
sw_major = bits(p, o, 8); o += 8
sw_minor = bits(p, o, 8); o += 8
o += 8 + 32 + 64            # optional flags, vcs commit, image crc
hw_major = bits(p, o, 8); o += 8
hw_minor = bits(p, o, 8); o += 8
uid = (int.from_bytes(p, "little") >> o & ((1 << 128) - 1)).to_bytes(16, "little"); o += 128
cert_len = bits(p, o, 8); o += 8 + cert_len * 8
name = p[o // 8:].decode(errors="replace")
print(f"CRC {'OK' if crc_ok else 'BAD'}; uptime {uptime} s, health {['OK', 'WARNING', 'ERROR', 'CRITICAL'][health]}, mode {mode}, vendor status {vendor}")
print(f"software {sw_major}.{sw_minor}, hardware {hw_major}.{hw_minor}, unique id {uid.hex()}")
print(f"name: {name!r}")
