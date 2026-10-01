# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
file(GLOB hal_source ${stm32_g0xx_SOURCE_DIR}/Src/*.c)

list(FILTER hal_source EXCLUDE REGEX "template\\.c$")

add_library(stm32cubeg0 STATIC ${hal_source})

target_include_directories(stm32cubeg0 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
    ${CMAKE_BINARY_DIR}/stm32_g0xx/Inc/
    # Remove these for fetch content
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Device/ST/STM32G0xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/Drivers/CMSIS/Include
)

target_compile_definitions(stm32cubeg0 PUBLIC STM32G0B1xx)

file(GLOB source 
  ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.c
  ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.cpp
  ${CMAKE_CURRENT_LIST_DIR}/Core/Startup/*.s
)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

include(${CMAKE_SOURCE_DIR}/cmake/dronecan_c.cmake)

add_executable(${PROJECT_NAME} ${source} ${CANARD_SRC_DIR}/canard.c ${DSDLC_SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME}.elf)

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}/board
    ${CANARD_SRC_DIR}
    ${DSDLC_GEN_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/Core/Inc
)


target_compile_options(${PROJECT_NAME} PRIVATE
    --specs=nano.specs
    -ffunction-sections
    -fdata-sections
    -Os
)

target_link_options(${PROJECT_NAME} PRIVATE
    -T ${CMAKE_SOURCE_DIR}/stm_generated/STM32G0B1KBTX_FLASH.ld
    -Wl,--gc-sections
    -Wl,-Map=firmware.map
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  stm32cubeg0
)

set(PROGRAMMER openocd)
set(PROGRAMMER_FLAGS -f interface/stlink.cfg -f target/stm32g0x.cfg)

add_custom_target(flash
    COMMAND ${PROGRAMMER} ${PROGRAMMER_FLAGS} -c "program $<TARGET_FILE:${PROJECT_NAME}> verify reset exit"
    DEPENDS ${PROJECT_NAME}
    VERBATIM)
  
