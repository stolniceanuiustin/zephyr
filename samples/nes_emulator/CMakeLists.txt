cmake_minimum_required(VERSION 3.20.0)
set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/app.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nes_emulator CXX C)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
    src/main.cpp
    src/virtual_screen.cpp
    src/CPU/cpu_core.cpp
    src/CPU/cpu_instructions.cpp
    src/CPU/cpu_utils.cpp
    src/CPU/address_logic.cpp
    src/PPU/PPU.cpp
    src/PPU/ppu_helper.cpp
    src/BUSComponents/bus.cpp
    src/BUSComponents/memory.cpp
    src/BUSComponents/cartridge.cpp
    src/BUSComponents/GLOBAL_VARIABLES.cpp
    src/input/nes_controller.cpp
    src/fb_udp.c
)

# Embed the NES ROM directly into the firmware image as a C byte array.
# Override with -DNES_ROM=/abs/path/to/game.nes at configure time.
# Default points at the smb.bin used during development.
set(NES_ROM "$ENV{HOME}/ZephyrOpensource/ADI_Temp/Licenta/smb.bin"
    CACHE FILEPATH "Path to the NES ROM to embed in the firmware")

if(NOT EXISTS "${NES_ROM}")
    message(FATAL_ERROR "NES_ROM not found: ${NES_ROM}\n"
                        "Pass -DNES_ROM=/path/to/game.nes to west build.")
endif()

# Write into include/generated (already on the system include path) so the
# array can be pulled in with #include <nes_rom.inc>.
generate_inc_file_for_target(app "${NES_ROM}"
    ${ZEPHYR_BINARY_DIR}/include/generated/nes_rom.inc)
