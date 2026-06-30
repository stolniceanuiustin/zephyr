#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "memory.h"
#include "cartridge.h"

/* The NES ROM is embedded directly in the firmware image (rodata) by CMake
 * via generate_inc_file_for_target() — see CMakeLists.txt. No SD load, no
 * fixed address to coordinate; the bytes are always present and mapped. */
static const uint8_t rom_buf[] = {
#include <nes_rom.inc>
};
#define ROM_BUF_SIZE (sizeof(rom_buf))

LOG_MODULE_REGISTER(cartridge, LOG_LEVEL_INF);

Nametable_Map nametablee;
Config config;
MIRROR mirroring;

static void set_mapping(uint16_t top_left, uint16_t top_right, uint16_t bottom_left, uint16_t bottom_right)
{
    nametablee.map[0] = top_left;
    nametablee.map[1] = top_right;
    nametablee.map[2] = bottom_left;
    nametablee.map[3] = bottom_right;
}

bool cartridge_read_file(const char *rom_name)
{
    ARG_UNUSED(rom_name);

    const uint8_t *rom = rom_buf;
    LOG_INF("Loading ROM from buffer at %p", (void *)rom);

    const NESHeader *header = (const NESHeader *)rom;

    if (header->magic[0] != 'N' || header->magic[1] != 'E' ||
        header->magic[2] != 'S' || header->magic[3] != 0x1A) {
        LOG_ERR("Invalid NES file: missing iNES magic");
        return false;
    }

    LOG_INF("flags6=0x%02x flags7=0x%02x", header->flags6, header->flags7);

    const uint8_t *data = rom + sizeof(NESHeader);

    if (header->flags6 & 0x04) {
        LOG_INF("Skipping 512-byte trainer");
        data += 512;
    }

    byte mapper_type = ((header->flags6 & 0b11110000) >> 4) | (header->flags7 & 0b11110000);
    LOG_INF("Mapper type: %d", mapper_type);

    byte nametable_type = (header->flags6 & 1);
    if (nametable_type == 0) {
        LOG_INF("Vertical mirroring");
        mirroring = VERTICAL;
        set_mapping(0, 0x400, 0, 0x400);
    } else {
        LOG_INF("Horizontal mirroring");
        mirroring = HORIZONTAL;
        set_mapping(0, 0, 0x400, 0x400);
    }
    config.nametable_arrangement = (nametable_type == 1) ? 1 : 0;

    if (mapper_type == 0) {
        LOG_INF("Mapper 0 (NROM)");
        int prg_size = header->prg_size * PRG_BANK_SIZE;
        int chr_size = header->chr_size * CHR_BANK_SIZE;

        memcpy(PRGrom, data, prg_size);
        data += prg_size;

        if (prg_size == PRG_BANK_SIZE) {
            LOG_INF("NROM-128: mirroring 16KB PRG");
            memcpy(PRGrom + PRG_BANK_SIZE, PRGrom, PRG_BANK_SIZE);
        }
        if (chr_size > 0 && chr_size <= CHR_BANK_SIZE) {
            memcpy(CHRrom, data, chr_size);
        } else {
            LOG_WRN("CHR-RAM present (mapper mismatch or CHR-RAM game)");
        }
    } else {
        LOG_ERR("Unsupported mapper: %d", mapper_type);
        return false;
    }

    LOG_INF("ROM loaded successfully");
    return true;
}
