#ifndef VIRTUAL_SCREEN_H
#define VIRTUAL_SCREEN_H
#include <stdint.h>

/*
This file contains a virtual screen from which
the framebuffer will be created.

Each pixel is stored as a 6-bit NES palette index (1 byte/pixel) rather than a
resolved ARGB32 color. This keeps the framebuffer at 256*240 = 60 KB instead of
240 KB, cutting the per-frame TCP payload by 4x. The PC viewer owns the
palette->RGB lookup (see pc_viewer/viewer.py, which mirrors the table below).
*/
void screen_init();
extern uint8_t pixels[256 * 240];
extern volatile bool RENDER_ENABLED;

inline void screen_set_pixel(uint16_t scanline, uint16_t dot, uint8_t color_index)
{
    pixels[256 * scanline + dot] = color_index;
}


#endif