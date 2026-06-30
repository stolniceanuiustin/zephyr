#ifndef NES_CONTROLLER_H
#define NES_CONTROLLER_H

#include <stdint.h>

/*
 * Button bits in the returned byte (active high, unlike the raw wire).
 * Order matches the 74HC4021 shift order: A is MSB, Right is LSB.
 */
#define NES_BTN_A       (1 << 7)
#define NES_BTN_B       (1 << 6)
#define NES_BTN_SELECT  (1 << 5)
#define NES_BTN_START   (1 << 4)
#define NES_BTN_UP      (1 << 3)
#define NES_BTN_DOWN    (1 << 2)
#define NES_BTN_LEFT    (1 << 1)
#define NES_BTN_RIGHT   (1 << 0)

int  nes_controller_init(void);
uint8_t nes_controller_read(void);

#endif
