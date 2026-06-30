#ifndef FB_UDP_H
#define FB_UDP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int fb_udp_init(void);
void fb_udp_send_frame(const uint32_t *pixels, int width, int height);

#ifdef __cplusplus
}
#endif

#endif
