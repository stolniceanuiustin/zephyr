#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "memory.h"
#include "cartridge.h"
#include "cpu.h"

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

/* ================= MMC3 (mapper 4) ==================================
 *
 * Same copy-into-window strategy as MMC1, but finer granularity (8 KB PRG
 * slots, 1 KB CHR slots) and a scanline IRQ counter. The counter is clocked
 * once per visible scanline from the PPU (mapper_scanline()) rather than from
 * PPU A12 edges; that is accurate enough for the raster splits games use. */

#define PRG_SLOT_8K (8 * 1024)
#define CHR_SLOT_1K (1 * 1024)

/* Bank select register ($8000, even address). */
#define MMC3_SEL_REG_MASK 0x07 /* which of R0..R7 the next data write targets */
#define MMC3_SEL_PRG_MODE 0x40 /* bit 6: PRG bank layout */
#define MMC3_SEL_CHR_MODE 0x80 /* bit 7: CHR A12 inversion (swap 2K/1K halves) */

#define MMC3_MIRROR_HORIZONTAL 0x01 /* $A000 bit 0 */
#define MMC3_PRG_BANK_MASK     0x3F /* 6-bit 8 KB bank select */
#define MMC3_CHR_2K_MASK       0xFE /* 2 KB banks ignore the low 1 KB bit */
#define MMC3_CHR_INVERT_SLOTS  4    /* XOR on the 1 KB slot index flips regions */
#define MMC3_CHR_FIRST_1K_REG  2    /* R2 is the first 1 KB register */
#define CHR_TILES_PER_1K       64   /* 1024 bytes / 16 bytes per tile */

static const uint8_t *mmc3_prg_base; /* -> PRG image in rom_buf */
static const uint8_t *mmc3_chr_base; /* -> CHR image in rom_buf */
static uint16_t mmc3_prg_banks_8k;
static uint16_t mmc3_chr_banks_1k;

static uint8_t mmc3_regs[8];    /* R0..R7 bank registers */
static uint8_t mmc3_bank_select; /* last $8000 (even) write */
static uint8_t mmc3_irq_latch;   /* counter reload value */
static uint8_t mmc3_irq_counter;
static bool mmc3_irq_reload;
static bool mmc3_irq_enabled;

/* slot 0..3 -> $8000/$A000/$C000/$E000. */
static void mmc3_copy_prg(uint8_t bank8k, uint8_t slot)
{
    const uint8_t *src = mmc3_prg_base + (bank8k % mmc3_prg_banks_8k) * PRG_SLOT_8K;
    memcpy(PRGrom + slot * PRG_SLOT_8K, src, PRG_SLOT_8K);
}

/* slot 0..7 -> 1 KB windows across $0000-$1FFF. */
static void mmc3_copy_chr(uint16_t bank1k, uint8_t slot)
{
    const uint8_t *src = mmc3_chr_base + (bank1k % mmc3_chr_banks_1k) * CHR_SLOT_1K;
    memcpy(CHRrom + slot * CHR_SLOT_1K, src, CHR_SLOT_1K);
}

/* Re-decode the 64 tiles a 1 KB CHR slot maps to. Skipped while the cache is
 * uninitialised — the pending full rebuild at the next render covers it. */
static void mmc3_chr_decode_slot(uint8_t slot)
{
    if (tile_cache_initialized) {
        build_tile_cache_range(slot * CHR_TILES_PER_1K, CHR_TILES_PER_1K);
    }
}

/* Lay down all four PRG slots. $A000 (R7) and $E000 (last bank) are fixed;
 * $8000 and $C000 swap with the PRG mode bit. Just memcpy, no tile decode. */
static void mmc3_apply_prg(void)
{
    uint8_t last = mmc3_prg_banks_8k - 1;
    uint8_t penult = mmc3_prg_banks_8k - 2;

    if (mmc3_bank_select & MMC3_SEL_PRG_MODE) {
        mmc3_copy_prg(penult, 0);
        mmc3_copy_prg(mmc3_regs[7] & MMC3_PRG_BANK_MASK, 1);
        mmc3_copy_prg(mmc3_regs[6] & MMC3_PRG_BANK_MASK, 2);
        mmc3_copy_prg(last, 3);
    } else {
        mmc3_copy_prg(mmc3_regs[6] & MMC3_PRG_BANK_MASK, 0);
        mmc3_copy_prg(mmc3_regs[7] & MMC3_PRG_BANK_MASK, 1);
        mmc3_copy_prg(penult, 2);
        mmc3_copy_prg(last, 3);
    }
}

/* CHR mode bit swaps which half holds the two 2 KB banks (R0/R1) and which
 * holds the four 1 KB banks (R2..R5); XOR flips the 1 KB slot region. */
static uint8_t mmc3_chr_invert(void)
{
    return (mmc3_bank_select & MMC3_SEL_CHR_MODE) ? MMC3_CHR_INVERT_SLOTS : 0;
}

/* Full CHR re-lay: only needed on a mode-bit flip and at load. Defers the tile
 * decode to one rebuild at the next render. */
static void mmc3_apply_chr(void)
{
    if (chr_is_ram) {
        return;
    }

    uint8_t inv = mmc3_chr_invert();

    mmc3_copy_chr(mmc3_regs[0] & MMC3_CHR_2K_MASK, 0 ^ inv);
    mmc3_copy_chr((mmc3_regs[0] & MMC3_CHR_2K_MASK) + 1, 1 ^ inv);
    mmc3_copy_chr(mmc3_regs[1] & MMC3_CHR_2K_MASK, 2 ^ inv);
    mmc3_copy_chr((mmc3_regs[1] & MMC3_CHR_2K_MASK) + 1, 3 ^ inv);
    mmc3_copy_chr(mmc3_regs[2], 4 ^ inv);
    mmc3_copy_chr(mmc3_regs[3], 5 ^ inv);
    mmc3_copy_chr(mmc3_regs[4], 6 ^ inv);
    mmc3_copy_chr(mmc3_regs[5], 7 ^ inv);
    tile_cache_initialized = false;
}

/* Hot path: a single CHR register changed — copy and re-decode only its
 * slot(s), leaving the rest of the tile cache intact. */
static void mmc3_apply_chr_reg(uint8_t r)
{
    if (chr_is_ram) {
        return;
    }

    uint8_t inv = mmc3_chr_invert();

    if (r == 0 || r == 1) {
        uint8_t base_slot = (r == 0) ? 0 : 2;
        uint8_t bank = mmc3_regs[r] & MMC3_CHR_2K_MASK;
        mmc3_copy_chr(bank, base_slot ^ inv);
        mmc3_copy_chr(bank + 1, (base_slot + 1) ^ inv);
        mmc3_chr_decode_slot(base_slot ^ inv);
        mmc3_chr_decode_slot((base_slot + 1) ^ inv);
    } else {
        uint8_t slot = (r - MMC3_CHR_FIRST_1K_REG + 4) ^ inv;
        mmc3_copy_chr(mmc3_regs[r], slot);
        mmc3_chr_decode_slot(slot);
    }
}

void mmc3_write(uint16_t addr, byte data)
{
    bool odd = (addr & 1) != 0;

    if (addr <= 0x9FFF) { /* bank select / bank data */
        if (!odd) {
            /* Bank select changes only which register the next data write hits;
             * re-lay a bank space only if its mode bit actually flipped. */
            uint8_t changed = mmc3_bank_select ^ data;
            mmc3_bank_select = data;
            if (changed & MMC3_SEL_PRG_MODE) {
                mmc3_apply_prg();
            }
            if (changed & MMC3_SEL_CHR_MODE) {
                mmc3_apply_chr();
            }
        } else {
            uint8_t r = mmc3_bank_select & MMC3_SEL_REG_MASK;
            mmc3_regs[r] = data;
            if (r >= 6) {
                mmc3_apply_prg(); /* R6/R7 select PRG banks */
            } else {
                mmc3_apply_chr_reg(r); /* R0..R5 select CHR banks */
            }
        }
    } else if (addr <= 0xBFFF) { /* mirroring / PRG-RAM protect */
        if (!odd) {
            ppu_set_mirroring((data & MMC3_MIRROR_HORIZONTAL) ? NT_MIRROR_HORIZONTAL
                                                             : NT_MIRROR_VERTICAL);
        }
        /* odd: PRG-RAM protect bits — not emulated. */
    } else if (addr <= 0xDFFF) { /* IRQ latch / reload */
        if (!odd) {
            mmc3_irq_latch = data;
        } else {
            mmc3_irq_reload = true;
        }
    } else { /* $E000-$FFFF: IRQ disable+ack / enable */
        if (!odd) {
            mmc3_irq_enabled = false;
            pending_irq = false; /* acknowledge a pending line */
        } else {
            mmc3_irq_enabled = true;
        }
    }
}

void mapper_scanline(void)
{
    if (cart_mapper != MAPPER_MMC3) {
        return;
    }

    if (mmc3_irq_counter == 0 || mmc3_irq_reload) {
        mmc3_irq_counter = mmc3_irq_latch;
        mmc3_irq_reload = false;
    } else {
        mmc3_irq_counter--;
    }

    if (mmc3_irq_counter == 0 && mmc3_irq_enabled) {
        pending_irq = true;
    }
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
    } else if (mapper_type == MAPPER_MMC3) {
        LOG_INF("Mapper 4 (MMC3)");

        mmc3_prg_base = data;
        mmc3_prg_banks_8k = header->prg_size * (PRG_BANK_SIZE / PRG_SLOT_8K);
        data += header->prg_size * PRG_BANK_SIZE;

        if (header->chr_size > 0) {
            mmc3_chr_base = data;
            mmc3_chr_banks_1k = header->chr_size * (CHR_BANK_SIZE / CHR_SLOT_1K);
            chr_is_ram = false;
        } else {
            chr_is_ram = true; /* 8 KB writable CHR window, no banking */
            LOG_INF("MMC3 CHR-RAM game");
        }

        LOG_INF("MMC3: %u PRG banks (8K), %u CHR banks (1K), chr_ram=%d",
                mmc3_prg_banks_8k, mmc3_chr_banks_1k, (int)chr_is_ram);

        /* Power-on: registers zero, PRG mode 0. The fixed second-to-last/last
         * banks are placed at $C000/$E000 here so the reset vector is valid
         * before cpu_reset(). IRQ starts disabled. */
        memset(mmc3_regs, 0, sizeof(mmc3_regs));
        mmc3_bank_select = 0;
        mmc3_irq_latch = 0;
        mmc3_irq_counter = 0;
        mmc3_irq_reload = false;
        mmc3_irq_enabled = false;
        mmc3_apply_prg();
        mmc3_apply_chr();
    } else {
        LOG_ERR("Unsupported mapper: %d", mapper_type);
        return false;
    }

    LOG_INF("ROM loaded successfully");
    return true;
}
