# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
# Fetches libcanard and generates C code for the DroneCAN messages this firmware uses.
#
# Provides:
#   CANARD_SRC_DIR    libcanard sources (canard.c, canard.h)
#   DSDLC_GEN_DIR     generated include/ and src/ directories
#   DSDLC_SOURCES     generated .c files to compile
#
# Everything is pinned to a commit so builds are reproducible.

include(FetchContent)

FetchContent_Declare(libcanard
  GIT_REPOSITORY https://github.com/dronecan/libcanard.git
  GIT_TAG        601ed35467e0ac38819df17cd7c918de19f62d58
  SOURCE_DIR     ${CMAKE_BINARY_DIR}/libcanard)
FetchContent_Declare(dronecan_dsdlc
  GIT_REPOSITORY https://github.com/dronecan/dronecan_dsdlc.git
  GIT_TAG        431170fa4bfe2212b516b8f33bdc796267907f1c
  SOURCE_DIR     ${CMAKE_BINARY_DIR}/dronecan_dsdlc)
FetchContent_Declare(dronecan_dsdl
  GIT_REPOSITORY https://github.com/dronecan/DSDL.git
  GIT_TAG        b4653c7abc3c47cb31b16efa24ea755232774756
  SOURCE_DIR     ${CMAKE_BINARY_DIR}/DSDL)
FetchContent_Populate(libcanard)
FetchContent_Populate(dronecan_dsdlc)
FetchContent_Populate(dronecan_dsdl)

set(CANARD_SRC_DIR ${CMAKE_BINARY_DIR}/libcanard)
set(DSDLC_GEN_DIR ${CMAKE_BINARY_DIR}/dsdlc_generated)
set(VENV_DIR ${CMAKE_BINARY_DIR}/venv)

if(NOT EXISTS ${DSDLC_GEN_DIR}/include/dronecan_msgs.h)
  message(STATUS "Generating DroneCAN C code")
  execute_process(COMMAND python3 -m venv ${VENV_DIR} RESULT_VARIABLE rc)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "Failed to create python venv")
  endif()
  execute_process(COMMAND ${VENV_DIR}/bin/pip install -q empy==4.2.1 pexpect dronecan==1.0.27 RESULT_VARIABLE rc)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "Failed to install python requirements")
  endif()
  execute_process(
    COMMAND ${VENV_DIR}/bin/python ${CMAKE_BINARY_DIR}/dronecan_dsdlc/dronecan_dsdlc.py
            -O ${DSDLC_GEN_DIR} ${CMAKE_BINARY_DIR}/DSDL/uavcan
    RESULT_VARIABLE rc ERROR_VARIABLE err)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "Failed to generate DroneCAN C code\n${err}")
  endif()
endif()

set(DSDLC_SOURCES
  ${DSDLC_GEN_DIR}/src/uavcan.Timestamp.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.NodeStatus.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.SoftwareVersion.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.HardwareVersion.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.GetNodeInfo_res.c
  ${DSDLC_GEN_DIR}/src/uavcan.equipment.gnss.ECEFPositionVelocity.c
  ${DSDLC_GEN_DIR}/src/uavcan.equipment.gnss.Fix2.c
  ${DSDLC_GEN_DIR}/src/uavcan.equipment.gnss.Auxiliary.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.param.Empty.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.param.Value.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.param.NumericValue.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.param.GetSet_req.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.param.GetSet_res.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.param.ExecuteOpcode_req.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.param.ExecuteOpcode_res.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.RestartNode_req.c
  ${DSDLC_GEN_DIR}/src/uavcan.protocol.RestartNode_res.c)
