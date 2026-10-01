#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
"""Reads and sets the node ID over CAN through the serial USB-CAN adapter, using the reference
DroneCAN library (pip install dronecan pyserial) to build the requests and decode the replies.

Usage: can_param.py [-d /dev/ttyUSB0] NODE_ID command [value]

  get              show the NODE_ID parameter (0 = not set, the ID jumpers decide)
  set VALUE        store a node ID (1-127), or 0 to go back to the jumpers; applied after a restart
  list             list the parameters by index
  erase            clear the stored node ID (same as set 0)
  restart          restart the node (applies a new node ID)
"""
import argparse
import sys
import time

import dronecan
import dronecan.transport as transport
import serial

SELF = 50  # our own node ID on the bus

ap = argparse.ArgumentParser(add_help=False)
ap.add_argument("-d", "--device", default="/dev/ttyUSB0")
ap.add_argument("node", type=int)
ap.add_argument("command", choices=["get", "set", "list", "erase", "restart"])
ap.add_argument("value", nargs="?", type=int)
ap.add_argument("--repeat", type=int, default=1, help="with get: send the request this many times and count the replies")
args = ap.parse_args()

port = serial.Serial(None, 2000000, timeout=0.02, stopbits=serial.STOPBITS_TWO)
port.port = args.device
port.dtr = True   # the adapter only streams while DTR and RTS are asserted
port.rts = True
port.open()
cfg = bytearray([0xAA, 0x55, 0x12, 0x01, 0x01] + [0] * 8 + [0x00, 0x01, 0, 0, 0, 0])  # 1 Mbit/s, normal
cfg.append(sum(cfg[2:]) & 0xFF)
port.write(cfg)
time.sleep(0.3)
port.reset_input_buffer()

manager = transport.TransferManager()
buf = bytearray()
# Start at a different transfer ID each run: a receiver drops a repeat of the previous transfer ID that
# arrives within a couple of seconds as a duplicate
transfer_id = int(time.time() * 7) & 31


def send(payload):
    """Sends a service request and returns the reply payload, or None on timeout."""
    global transfer_id, buf
    tr = transport.Transfer(transfer_id=transfer_id, source_node_id=SELF, dest_node_id=args.node,
                            payload=payload, transfer_priority=20,
                            request_not_response=True, service_not_message=True)
    transfer_id = (transfer_id + 1) & 31
    for frame in tr.to_frames():
        port.write(bytes([0xAA, 0xE0 | len(frame.bytes)]) + frame.message_id.to_bytes(4, "little") + bytes(frame.bytes) + b"\x55")
        time.sleep(0.003)   # the adapter drops frames sent back to back
    deadline = time.time() + 2
    while time.time() < deadline:
        buf += port.read(4096)
        i = 0
        while i < len(buf):
            if buf[i] == 0xAA and i + 1 < len(buf) and (buf[i + 1] & 0xC0) == 0xC0:
                end = i + 6 + (buf[i + 1] & 15)
                if end >= len(buf):
                    break
                if buf[end] == 0x55:
                    frame = transport.Frame(int.from_bytes(buf[i + 2:i + 6], "little"), bytes(buf[i + 6:end]))
                    frames = manager.receive_frame(frame)
                    if frames:
                        try:
                            done = transport.Transfer()
                            done.from_frames(frames)
                        except transport.TransferError:
                            done = None   # not a transfer we can decode, or a bad CRC
                        if done is not None and done.source_node_id == args.node and done.dest_node_id == SELF \
                                and not done.request_not_response:
                            del buf[:end + 1]
                            return done.payload
                    i = end + 1
                    continue
            i += 1
        del buf[:i]
    return None


def getset(index=0, name="", value=None):
    req = dronecan.uavcan.protocol.param.GetSet.Request(index=index, name=name)
    if value is not None:
        req.value = dronecan.uavcan.protocol.param.Value(integer_value=value)
    for _ in range(3):   # a frame can be lost on the bus or in the adapter; repeating a get or set is harmless
        res = send(req)
        if res is not None:
            return res
    return None


def show(res):
    if res is None:
        sys.exit("no reply")
    name = res.name.decode() if isinstance(res.name, (bytes, bytearray)) else str(res.name)
    if not name:
        print("(no such parameter)")
        return False
    kind = dronecan.get_active_union_field(res.value)
    print(f"{name} = {getattr(res.value, kind)}   default {getattr(res.default_value, dronecan.get_active_union_field(res.default_value))}"
          f"   min {res.min_value.integer_value}   max {res.max_value.integer_value}")
    return True


if args.command == "get" and args.repeat > 1:
    replies = sum(1 for _ in range(args.repeat) if send(dronecan.uavcan.protocol.param.GetSet.Request(name="NODE_ID")) is not None)
    print(f"{replies} of {args.repeat} get requests were answered")
elif args.command == "get":
    show(getset(name="NODE_ID"))
elif args.command == "list":
    index = 0
    while show(getset(index=index)):
        index += 1
elif args.command == "set":
    if args.value is None:
        sys.exit("set needs a value")
    show(getset(name="NODE_ID", value=args.value))
    print("(the new node ID is used after the node restarts)")
elif args.command == "erase":
    res = send(dronecan.uavcan.protocol.param.ExecuteOpcode.Request(opcode=1))
    print("erase:", "ok" if res is not None and res.ok else "failed" if res is not None else "no reply")
elif args.command == "restart":
    res = send(dronecan.uavcan.protocol.RestartNode.Request(magic_number=dronecan.uavcan.protocol.RestartNode.Request().MAGIC_NUMBER))
    print("restart:", "accepted" if res is not None and res.ok else "refused" if res is not None else "no reply")
