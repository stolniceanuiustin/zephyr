# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ad9081_chipid)

# ADI AD9081 API library (copied verbatim from no-OS, HAL-abstracted).
file(GLOB ADI_API_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/adi_api/*.c)

target_sources(app PRIVATE
  src/main.c
  src/spi_mmio_fixup.c
  src/ad9081.c
  src/jesd_fsm.c
)

# Fault-injection tests: the failure/recovery paths a working board never takes.
# Off by default -- the tests break a live link on purpose.
target_sources_ifdef(CONFIG_AD9081_FAULT_INJECTION app PRIVATE
  src/fault_injection.c
)

target_sources(app PRIVATE
  ${ADI_API_SRCS}
)

target_include_directories(app PRIVATE src src/adi_api)
