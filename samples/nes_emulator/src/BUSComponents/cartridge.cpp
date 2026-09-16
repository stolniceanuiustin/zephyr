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
byte cart_mapper;
bool chr_is_ram;

/* ================= MMC1 (mapper 1) ==================================
 *
 * The full PRG/CHR image lives in rodata (rom_buf). Rather than teach the CPU
 * and PPU read paths about banking, the mapper copies the currently selected
 * banks into the fixed PRGrom (32 KB) / CHRrom (8 KB) windows on every switch.
 * Bank switches happen a handful of times per frame at most, so the copies are
 * cheap and the hot read paths stay untouched and reviewable. */

/* MMC1 serial port: five writes shift one value in, LSB first. */
#define MMC1_SHIFT_RESET_BIT 0x80 /* bit 7: reset shift register */
#define MMC1_SHIFT_INITIAL   0x10 /* sentinel bit reaching bit 0 marks 5th write */
#define MMC1_DATA_BIT        0x01 /* bit 0: the payload bit */

/* Control register ($8000-$9FFF) fields. */
#define MMC1_CTRL_MIRROR_MASK   0x03
#define MMC1_CTRL_PRG_MODE_MASK 0x0C
#define MMC1_CTRL_PRG_MODE_POS  2
#define MMC1_CTRL_CHR_MODE_4K   0x10 /* 1: two 4 KB banks, 0: one 8 KB bank */

/* PRG banking modes (control bits 3-2). */
#define MMC1_PRG_MODE_32K_LOW  0 /* 32 KB switch, ignore low bank bit */
#define MMC1_PRG_MODE_32K_HIGH 1 /* same as mode 0 */
#define MMC1_PRG_MODE_FIX_FIRST 2 /* fix bank 0 at $8000, switch $C000 */
#define MMC1_PRG_MODE_FIX_LAST  3 /* switch $8000, fix last bank at $C000 */

/* Power-on state: PRG mode 3 so the reset vector in the last bank is at $C000
 * before the CPU reset vector is fetched. */
#define MMC1_CTRL_RESET_STATE 0x0C

#define MMC1_PRG_BANK_MASK 0x0F /* 4-bit 16 KB bank select */
#define MMC1_CHR_BANK_MASK 0x1F /* 5-bit 4 KB bank select */

#define PRG_SLOT_SIZE (16 * 1024)
#define CHR_SLOT_SIZE (4 * 1024) /* MMC1 CHR bank granularity */

static const uint8_t *mmc1_prg_base; /* -> PRG image in rom_buf */
static const uint8_t *mmc1_chr_base; /* -> CHR image in rom_buf */
static uint8_t mmc1_prg_banks;       /* count of 16 KB PRG banks */
static uint8_t mmc1_chr_banks;       /* count of 4 KB CHR banks */

static uint8_t mmc1_shift = MMC1_SHIFT_INITIAL;
static uint8_t mmc1_control = MMC1_CTRL_RESET_STATE;
static uint8_t mmc1_chr0, mmc1_chr1, mmc1_prg;

/* slot 0 -> $8000, slot 1 -> $C000. Modulo guards a bad bank index. */
static void mmc1_copy_prg(uint8_t bank, uint8_t slot)
{
    const uint8_t *src = mmc1_prg_base + (bank % mmc1_prg_banks) * PRG_SLOT_SIZE;
    memcpy(PRGrom + slot * PRG_SLOT_SIZE, src, PRG_SLOT_SIZE);
}

/* slot 0 -> $0000, slot 1 -> $1000 in the PPU CHR window. */
static void mmc1_copy_chr(uint8_t bank, uint8_t slot)
{
    const uint8_t *src = mmc1_chr_base + (bank % mmc1_chr_banks) * CHR_SLOT_SIZE;
    memcpy(CHRrom + slot * CHR_SLOT_SIZE, src, CHR_SLOT_SIZE);
}

static void mmc1_apply_banks(void)
{
    uint8_t prg_mode = (mmc1_control & MMC1_CTRL_PRG_MODE_MASK) >> MMC1_CTRL_PRG_MODE_POS;
    uint8_t prg_sel = mmc1_prg & MMC1_PRG_BANK_MASK;

    if (prg_mode == MMC1_PRG_MODE_FIX_FIRST) {
        mmc1_copy_prg(0, 0);
        mmc1_copy_prg(prg_sel, 1);
    } else if (prg_mode == MMC1_PRG_MODE_FIX_LAST) {
        mmc1_copy_prg(prg_sel, 0);
        mmc1_copy_prg(mmc1_prg_banks - 1, 1);
    } else {
        uint8_t base = prg_sel & 0x0E; /* 32 KB pair, ignore low bit */
        mmc1_copy_prg(base, 0);
        mmc1_copy_prg(base + 1, 1);
    }

    if (!chr_is_ram) {
        if (mmc1_control & MMC1_CTRL_CHR_MODE_4K) {
            mmc1_copy_chr(mmc1_chr0 & MMC1_CHR_BANK_MASK, 0);
            mmc1_copy_chr(mmc1_chr1 & MMC1_CHR_BANK_MASK, 1);
        } else {
            uint8_t base = mmc1_chr0 & (MMC1_CHR_BANK_MASK & 0x1E); /* 8 KB pair */
            mmc1_copy_chr(base, 0);
            mmc1_copy_chr(base + 1, 1);
        }
        tile_cache_initialized = false; /* PPU rebuilds from the new CHR window */
    }
}

void mmc1_write(uint16_t addr, byte data)
{
    if (data & MMC1_SHIFT_RESET_BIT) {
        mmc1_shift = MMC1_SHIFT_INITIAL;
        mmc1_control |= MMC1_CTRL_PRG_MODE_MASK; /* force PRG mode 3 */
        mmc1_apply_banks();
        return;
    }

    bool complete = (mmc1_shift & MMC1_DATA_BIT) != 0; /* sentinel reached bit 0 */
    mmc1_shift = (mmc1_shift >> 1) | ((data & MMC1_DATA_BIT) << 4);

    if (!complete) {
        return;
    }

    uint8_t value = mmc1_shift & MMC1_CHR_BANK_MASK;
    uint8_t reg = (addr >> 13) & 0x03; /* bits 14-13 select the register */

    switch (reg) {
    case 0:
        mmc1_control = value;
        ppu_set_mirroring(value & MMC1_CTRL_MIRROR_MASK);
        break;
    case 1:
        mmc1_chr0 = value;
        break;
    case 2:
        mmc1_chr1 = value;
        break;
    case 3:
        mmc1_prg = value;
        break;
    }

    mmc1_shift = MMC1_SHIFT_INITIAL;
    mmc1_apply_banks();
}

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
    cart_mapper = mapper_type;

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

    if (mapper_type == MAPPER_NROM) {
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
            chr_is_ram = true;
            LOG_WRN("CHR-RAM present (mapper mismatch or CHR-RAM game)");
        }
    } else if (mapper_type == MAPPER_MMC1) {
        LOG_INF("Mapper 1 (MMC1)");

        mmc1_prg_base = data;
        mmc1_prg_banks = header->prg_size; /* 16 KB units */
        data += header->prg_size * PRG_BANK_SIZE;

        if (header->chr_size > 0) {
            mmc1_chr_base = data;
            mmc1_chr_banks = header->chr_size * (CHR_BANK_SIZE / CHR_SLOT_SIZE);
            chr_is_ram = false;
        } else {
            chr_is_ram = true; /* CHR is the 8 KB writable window; no banking */
            LOG_INF("MMC1 CHR-RAM game");
        }

        LOG_INF("MMC1: %u PRG banks (16K), %u CHR banks (4K), chr_ram=%d",
                mmc1_prg_banks, mmc1_chr_banks, (int)chr_is_ram);

        /* Power-on: shift register empty, control mode 3 (last bank fixed at
         * $C000). Lay down the initial banks so the CPU reset vector is valid. */
        mmc1_shift = MMC1_SHIFT_INITIAL;
        mmc1_control = MMC1_CTRL_RESET_STATE;
        mmc1_chr0 = mmc1_chr1 = mmc1_prg = 0;
        mmc1_apply_banks();
    } else {
        LOG_ERR("Unsupported mapper: %d", mapper_type);
        return false;
    }

    LOG_INF("ROM loaded successfully");
    return true;
}
