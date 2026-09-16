#ifndef CARTRIDGE_HEADERGUARD
#define CARTRIDGE_HEADERGUARD
#include <stdint.h>
#include "emulator_config.h"
#include "memory.h"
typedef uint8_t byte;

struct Nametable_Map
{
    uint16_t map[4];
};

enum MIRROR
{
    VERTICAL,
    HORIZONTAL
};
extern Config config;


extern MIRROR mirroring;

bool cartridge_read_file(const char* rom_name);
const int PRG_BANK_SIZE = 16 * 1024;
const int CHR_BANK_SIZE = 8 * 1024;

/* iNES mapper numbers this emulator understands. */
#define MAPPER_NROM 0
#define MAPPER_MMC1 1

/* Set by cartridge_read_file() so the bus (memory.cpp) can route CPU writes
 * to $8000-$FFFF into the active mapper instead of treating them as ROM. */
extern byte cart_mapper;

/* True when the cartridge has no CHR ROM and the 8 KB CHR window is RAM
 * (writable via the PPU bus). Mapper-0 games with CHR ROM leave this false. */
extern bool chr_is_ram;

/**
 * @brief Feed one CPU write to the MMC1 (mapper 1) serial port.
 *
 * MMC1 latches a value over five consecutive writes to $8000-$FFFF; this
 * drives that shift register and applies bank/mirroring changes once a
 * register completes. Only call for cart_mapper == MAPPER_MMC1.
 *
 * @param addr CPU address of the write ($8000-$FFFF); bits 14-13 select the
 *             target internal register on the fifth write.
 * @param data Byte written; bit 7 resets the shift register, bit 0 is data.
 */
void mmc1_write(uint16_t addr, byte data);




#endif