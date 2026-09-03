# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ad9081_chipid)

target_sources(app PRIVATE
  src/main.c
  src/ad9081_bringup.c
  src/jesd204_fsm.c
)

target_include_directories(app PRIVATE src)
