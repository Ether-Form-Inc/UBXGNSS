# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
cmake_minimum_required(VERSION 3.15)
include(FetchContent)

# STM  builds the firmware for the STM32G0B1 (needs arm-none-eabi-gcc).
# HOST builds the unit tests for the PC.
set(ARCH "STM" CACHE STRING "Target architecture: STM or HOST")

if(${ARCH} STREQUAL "STM")
  set(CMAKE_TOOLCHAIN_FILE ../stm32-toolchain.cmake CACHE STRING "" FORCE)
  project(can_gps_firmware LANGUAGES C CXX ASM)
  FetchContent_Declare(
    stm32_g0xx
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32g0xx-hal-driver.git
    GIT_TAG 289ef6c8ff0922878e99d253f29be76851f8f28e
    SOURCE_DIR ${CMAKE_BINARY_DIR}/stm32_g0xx
  )
  FetchContent_Populate(stm32_g0xx)

  add_subdirectory(stm_generated)
elseif(${ARCH} STREQUAL "HOST")
  project(can_gps_host_tests LANGUAGES C CXX)
  add_subdirectory(tests)
else()
  message(FATAL_ERROR "Specified architecture type not supported. Use \"-DARCH=<STM,HOST>\".")
endif()
