# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ad9081_chipid)

# ADI AD9081 API library (copied verbatim from no-OS, HAL-abstracted).
file(GLOB ADI_API_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/adi_api/*.c)

target_sources(app PRIVATE
  src/main.c
  src/ad9081.c
  src/hmc7044.c
  src/axi_jesd.c
  ${ADI_API_SRCS}
)

target_include_directories(app PRIVATE src/adi_api)
