/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Regression test for the LPF3 watchdog ms-to-ticks conversion overflow.
 *
 * A window of 131072 ms overflowed the 32-bit tick conversion to exactly 0
 * ticks, and writing 0 to WDTCNT resets the SoC immediately (TRM 6.9.50).
 * With the fix, setup() arms a ~131 s timeout and the test completes.
 *
 * NOTE for the runner: on a broken driver this test resets the chip during
 * wdt_setup() (multi-boot); on a fixed driver the watchdog is left armed
 * ~131 s in the future, so reflash/reset the board after the run.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

ZTEST(wdt_lpf3_long_timeout, test_long_window_does_not_reset)
{
	const struct device *wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	struct wdt_timeout_cfg cfg = {
		.window = {
			.min = 0U,
			/* 131072 ms * 32768 == 2^32: wrapped to 0 ticks */
			.max = 131072U,
		},
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};
	int err;

	zassert_true(device_is_ready(wdt), "watchdog device not ready");

	err = wdt_install_timeout(wdt, &cfg);
	zassert_equal(err, 0, "install_timeout failed: %d", err);

	err = wdt_setup(wdt, 0);
	zassert_equal(err, 0, "setup failed: %d", err);

	/* The broken conversion resets the SoC here, before this returns */
	k_busy_wait(300 * USEC_PER_MSEC);

	err = wdt_feed(wdt, 0);
	zassert_equal(err, 0, "feed failed: %d", err);
}

ZTEST_SUITE(wdt_lpf3_long_timeout, NULL, NULL, NULL, NULL, NULL);
