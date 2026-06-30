#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nes_test, LOG_LEVEL_INF);

static const struct gpio_dt_spec latch =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), nes_latch_gpios);
static const struct gpio_dt_spec clk =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), nes_clock_gpios);
static const struct gpio_dt_spec data =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), nes_data_gpios);

#define LATCH_US 12
#define CLOCK_US  6

static uint8_t read_controller(void)
{
    uint8_t state = 0;

    gpio_pin_set_dt(&latch, 1);
    k_busy_wait(LATCH_US);
    gpio_pin_set_dt(&latch, 0);
    k_busy_wait(CLOCK_US);

    for (int i = 7; i >= 0; i--) {
        if (gpio_pin_get_dt(&data) == 0) {
            state |= (1 << i);
        }
        if (i > 0) {
            gpio_pin_set_dt(&clk, 0);
            k_busy_wait(CLOCK_US);
            gpio_pin_set_dt(&clk, 1);
            k_busy_wait(CLOCK_US);
        }
    }

    return state;
}

static void print_buttons(uint8_t state)
{
    /* Only print when something is pressed */
    if (state == 0) {
        return;
    }

    printk("Pressed: ");
    if (state & BIT(7)) printk("A ");
    if (state & BIT(6)) printk("B ");
    if (state & BIT(5)) printk("SELECT ");
    if (state & BIT(4)) printk("START ");
    if (state & BIT(3)) printk("UP ");
    if (state & BIT(2)) printk("DOWN ");
    if (state & BIT(1)) printk("LEFT ");
    if (state & BIT(0)) printk("RIGHT ");
    printk("\n");
}

int main(void)
{
    printk("NES Controller Test\n");

    if (!gpio_is_ready_dt(&latch) ||
        !gpio_is_ready_dt(&clk)   ||
        !gpio_is_ready_dt(&data)) {
        printk("ERROR: GPIO not ready\n");
        return -1;
    }

    gpio_pin_configure_dt(&latch, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&clk,   GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&data,  GPIO_INPUT);

    printk("Ready — press buttons on the NES controller\n");

    uint8_t prev = 0;

    while (1) {
        uint8_t state = read_controller();

        /* Only print on change to avoid spamming the console */
        if (state != prev) {
            print_buttons(state);
            prev = state;
        }

        k_msleep(16);
    }

    return 0;
}
