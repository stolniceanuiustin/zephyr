#include "virtual_screen.h"
#include <string.h>

uint8_t pixels[256 * 240];
volatile bool RENDER_ENABLED;

void screen_init(void)
{
    memset(pixels, 0, sizeof(pixels));
    RENDER_ENABLED = false;
}
