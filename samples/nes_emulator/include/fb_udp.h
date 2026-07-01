#ifndef FB_UDP_H
#define FB_UDP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int fb_udp_init(void);
/* pixels is one byte per pixel (6-bit NES palette index); width*height bytes. */
void fb_udp_send_frame(const uint8_t *pixels, int width, int height);

#ifdef __cplusplus
}
#endif

#endif
