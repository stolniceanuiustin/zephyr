/*
 * Thin platform shim so the verbatim ADI/Xilinx GT reconfiguration driver
 * (xilinx_transceiver.c, copied unchanged from no-OS) builds on Zephyr.
 *
 * xilinx_transceiver.c depends on only a handful of no-OS symbols:
 *   - NO_OS_ARRAY_SIZE / NO_OS_BIT / NO_OS_DIV_ROUND_CLOSEST_ULL  (no_os_util.h)
 *   - pr_err / pr_debug                                           (no_os_print_log.h)
 *   - -EINVAL                                                     (no_os_error.h)
 *   - struct adxcvr + adxcvr_drp_read/adxcvr_drp_write            (axi_adxcvr.h)
 * We provide exactly those here, backed by Zephyr's logging, and let the DRP
 * accessors live in our own axi_adxcvr.c (plain AXI MMIO). This keeps the fiddly
 * VCO/divider math in xilinx_transceiver.c byte-for-byte identical to the vendor
 * source -- no hand-transcription of the register encodings.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef XCVR_SHIM_H_
#define XCVR_SHIM_H_

#include <errno.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

/* no_os_util.h subset used by xilinx_transceiver.c */
#define NO_OS_BIT(x)                     BIT(x)
#define NO_OS_ARRAY_SIZE(x)              ARRAY_SIZE(x)
#define NO_OS_DIV_ROUND_CLOSEST(x, y)    (((x) + (y) / 2) / (y))
#define NO_OS_DIV_ROUND_CLOSEST_ULL(x, y) NO_OS_DIV_ROUND_CLOSEST(x, y)

/*
 * no_os_print_log.h subset. The verbatim file uses pr_err/pr_debug with
 * printf-style args. Route them through printk so the file needs no per-TU
 * LOG_MODULE registration (keeping it byte-identical to vendor source).
 * pr_debug is a no-op -- the divider math is chatty and only useful when
 * actively debugging DRP writes.
 */
#define pr_err(fmt, ...)   printk("xcvr: " fmt, ##__VA_ARGS__)
#define pr_debug(fmt, ...) ((void)0)
#define pr_info(fmt, ...)  printk("xcvr: " fmt, ##__VA_ARGS__)

/*
 * axi_adxcvr.h subset: xilinx_transceiver.c only ever holds a struct adxcvr *
 * opaquely and passes it to the DRP accessors, so a forward declaration is all
 * it needs from us. The real definition lives in axi_adxcvr.c.
 */
struct adxcvr;

int adxcvr_drp_read(struct adxcvr *xcvr, unsigned int drp_port,
		    unsigned int reg, unsigned int *val);
int adxcvr_drp_write(struct adxcvr *xcvr, unsigned int drp_port,
		     unsigned int reg, unsigned int val);

#endif /* XCVR_SHIM_H_ */
