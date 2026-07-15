/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * pm-4 regression: sys_reboot(SYS_REBOOT_COLD) must actually reset the SoC.
 * Before the fix, SYS_REBOOT_COLD was a no-op and sys_reboot() hung in the
 * "Failed to reboot: spinning endlessly..." loop, so this test would time out.
 *
 * A magic-guarded __noinit counter survives a PMCTL system reset (SRAM is
 * retained across all resets except POR, TRM SWCU193A 3.x) but not a power-on
 * reset, so a fresh POR starts the sequence cleanly.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/reboot.h>

#define BOOT_MAGIC 0x5A5AC01DU

static __noinit uint32_t boot_magic;
static __noinit uint32_t boot_count;

ZTEST(reboot_cold, test_cold_reboot_resets)
{
	if (boot_magic != BOOT_MAGIC) {
		/* First boot after POR/flash: arm the sequence and cold-reboot. */
		boot_magic = BOOT_MAGIC;
		boot_count = 1;

		printk("reboot_cold: first boot, issuing SYS_REBOOT_COLD\n");
		sys_reboot(SYS_REBOOT_COLD);

		/* Only reached if the reboot did nothing (the bug). */
		zassert_unreachable("SYS_REBOOT_COLD did not reset the SoC");
	}

	/* Second boot: the cold reboot worked. */
	boot_count++;
	printk("reboot_cold: rebooted successfully, boot_count=%u\n", boot_count);
	zassert_equal(boot_count, 2, "unexpected boot count %u", boot_count);

	boot_magic = 0;
}

ZTEST_SUITE(reboot_cold, NULL, NULL, NULL, NULL, NULL);
