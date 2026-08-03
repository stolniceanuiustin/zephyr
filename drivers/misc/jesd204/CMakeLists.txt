# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_JESD204_AXI_ADXCVR
  axi_adxcvr.c
  xilinx_transceiver.c
)
zephyr_library_sources_ifdef(CONFIG_JESD204_AXI_LINK axi_jesd204.c)
zephyr_library_sources_ifdef(CONFIG_JESD204_AXI_TPL axi_tpl.c)

zephyr_library_include_directories(.)
