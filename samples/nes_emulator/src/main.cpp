#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bus.h"
#include "cartridge.h"
#include "cpu.h"
#include "ppu.h"
#include "virtual_screen.h"
#include "nes_controller.h"
#include "fb_udp.h"

LOG_MODULE_REGISTER(nes_main, LOG_LEVEL_INF);

#define ROM_PATH "" /* ROM loaded by U-Boot into RAM, path unused */

#define EMU_STACK_SIZE    8192
#define INPUT_STACK_SIZE  1024
#define EMU_PRIORITY      5
#define INPUT_PRIORITY    4   /* higher priority than emulator so input is always fresh */

extern uint32_t pixels[256 * 240];
extern volatile bool RENDER_ENABLED;

/* Shared controller byte — input thread writes, emulator thread reads */
static volatile uint8_t controller_byte;

/* Gate the emulator until the network is up. The emu thread busy-spins and
 * would otherwise starve the lower-priority net stack threads (PHY link poll,
 * net-mgmt, config), so the interface would never come up. Set true only after
 * fb_udp_init() has the interface ready. */
static volatile bool net_ready;

/* ------------------------------------------------------------------ */
/* Input thread: polls the NES controller every 10ms                  */
/* ------------------------------------------------------------------ */

static void input_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        controller_byte = nes_controller_read();
        k_msleep(10);
    }
}

K_THREAD_DEFINE(input_thread, INPUT_STACK_SIZE,
                input_thread_fn, NULL, NULL, NULL,
                INPUT_PRIORITY, 0, 0);

/* ------------------------------------------------------------------ */
/* Emulation thread                                                    */
/* ------------------------------------------------------------------ */

static bool leap_cycle;

static void emu_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Emulation thread started");

    /* Wait for networking to come up before busy-spinning, otherwise this
     * thread starves the net stack and the interface never initialises.
     * k_msleep() yields to all lower-priority threads (unlike k_yield()). */
    while (!net_ready) {
        k_msleep(50);
    }
    LOG_INF("Network ready — emulation running");

    uint32_t emu_us = 0, send_us = 0;
    int prof = 0;

    while (1) {
        uint32_t t0 = k_cycle_get_32();

        for (int i = 0; i <= 500; i++) {
            ppu_render_scanline();

            int cpu_ticks = 113 + (leap_cycle ? 1 : 0);
            leap_cycle = !leap_cycle;

            for (int j = 0; j < cpu_ticks; j++) {
                cpu_clock();
            }
        }

        controller[0] = controller_byte;

        uint32_t t1 = k_cycle_get_32();

        if (RENDER_ENABLED) {
            fb_udp_send_frame(pixels, 256, 240);
            RENDER_ENABLED = false;
        }

        uint32_t t2 = k_cycle_get_32();

        emu_us += k_cyc_to_us_floor32(t1 - t0);
        send_us += k_cyc_to_us_floor32(t2 - t1);
        if (++prof >= 60) {
            LOG_INF("avg/frame: emu=%u us  send=%u us  (~%u fps)",
                    emu_us / 60, send_us / 60,
                    1000000U / ((emu_us + send_us) / 60));
            emu_us = send_us = 0;
            prof = 0;
        }

        k_yield();
    }
}

K_THREAD_DEFINE(emu_thread, EMU_STACK_SIZE,
                emu_thread_fn, NULL, NULL, NULL,
                EMU_PRIORITY, 0, 0);

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    LOG_INF("NES Emulator starting on Zephyr");

    if (nes_controller_init() != 0) {
        LOG_WRN("NES controller init failed — running without input");
    }

    screen_init();
    memory_init();

    if (!cartridge_read_file(ROM_PATH)) {
        LOG_ERR("Failed to load ROM: %s", ROM_PATH);
        return -1;
    }

    bus_init();
    cpu_init();
    cpu_reset();
    ppu_init();

    if (fb_udp_init() == 0) {
        net_ready = true;
    } else {
        LOG_ERR("network init failed — framebuffer streaming disabled");
        /* Let the emulator run anyway so input/PPU still work locally. */
        net_ready = true;
    }

    LOG_INF("Emulator initialised — threads running");
    return 0;
}
