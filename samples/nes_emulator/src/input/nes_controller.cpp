#include "nes_controller.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nes_ctrl, LOG_LEVEL_INF);

/*
 * Pin mapping via devicetree. Add to your board overlay:
 *
 *   / {
 *       zephyr,user {
 *           nes-latch-gpios = <&gpio0 54 GPIO_ACTIVE_HIGH>;
 *           nes-clock-gpios = <&gpio0 55 GPIO_ACTIVE_HIGH>;
 *           nes-data-gpios  = <&gpio0 56 GPIO_ACTIVE_HIGH>;
 *       };
 *   };
 *
 * Pin numbers 54+ are the EMIO range on Zynq (MIO = 0-53).
 * Exact numbers depend on your Vivado EMIO GPIO assignment.
 */
static const struct gpio_dt_spec latch =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), nes_latch_gpios);
static const struct gpio_dt_spec clk =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), nes_clock_gpios);
static const struct gpio_dt_spec data =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), nes_data_gpios);

/* Latch must be held high >= 12µs for the 4021 to parallel-load */
#define LATCH_US 12
/* Half-period of the clock pulse — 6µs gives ~83kHz, well within spec */
#define CLOCK_US  6

int nes_controller_init(void)
{
    if (!gpio_is_ready_dt(&latch) ||
        !gpio_is_ready_dt(&clk)   ||
        !gpio_is_ready_dt(&data)) {
        LOG_ERR("NES controller GPIO device not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&latch, GPIO_OUTPUT_INACTIVE); /* LATCH starts LOW  */
    gpio_pin_configure_dt(&clk,   GPIO_OUTPUT_ACTIVE);   /* CLK starts HIGH   */
    gpio_pin_configure_dt(&data,  GPIO_INPUT);

    LOG_INF("NES controller initialised");
    return 0;
}

/*
 * Protocol (74HC4021 parallel-to-serial):
 *
 *  1. Pulse LATCH high for 12µs → all 8 button states loaded in parallel.
 *  2. LATCH low. First bit (A button) is already on Q8 — read it now.
 *  3. For each remaining 7 bits: CLK low → CLK high (rising edge shifts
 *     the register) → read the new Q8 value.
 *
 * The 4021 output is active-low because each button grounds its PI pin
 * through the button against the pull-up resistor. The NES hardware had
 * an inverter between Q8 and the CPU; we are reading the raw 4021 output
 * so we invert in software: 0 on wire = pressed = set bit.
 *
 * Button order, MSB first: A B Select Start Up Down Left Right
 */
uint8_t nes_controller_read(void)
{
    uint8_t state = 0;

    /* Parallel load */
    gpio_pin_set_dt(&latch, 1);
    k_busy_wait(LATCH_US);
    gpio_pin_set_dt(&latch, 0);
    k_busy_wait(CLOCK_US);

    for (int i = 7; i >= 0; i--) {
        /* Q8 is low when button is pressed (active-low, no NES inverter) */
        if (gpio_pin_get_dt(&data) == 0) {
            state |= (1 << i);
        }

        /* Rising edge shifts next button into Q8.
         * Skip after the last bit — nothing left to shift. */
        if (i > 0) {
            gpio_pin_set_dt(&clk, 0);
            k_busy_wait(CLOCK_US);
            gpio_pin_set_dt(&clk, 1);
            k_busy_wait(CLOCK_US);
        }
    }

    return state;
}
