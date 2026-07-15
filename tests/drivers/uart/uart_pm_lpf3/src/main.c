/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 * SPDX-License-Identifier: Apache-2.0
 *
 * LPF3 UART device-PM smoke test.
 *
 * Standby powers down the peripheral domain, so the UART's non-retained
 * registers (baud, line control, FIFO) are lost on suspend and reprogrammed on
 * resume by the driver's PM action (uart_lpf3_pm_action -> init_common). This
 * test drives the device-PM state machine directly (suspend/resume) and, after
 * each resume, confirms a byte still loops back DIO20->DIO22 -- i.e. the config
 * was actually restored, not just the PM state flipped.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <zephyr/ztest.h>

#define DUT DEVICE_DT_GET(DT_NODELABEL(uart0))

static bool loopback_byte(const struct device *dev, uint8_t tx)
{
	int64_t end = k_uptime_get() + 100;
	uint8_t rx;

	/* Drain any stale RX first. */
	while (uart_poll_in(dev, &rx) == 0) {
	}

	uart_poll_out(dev, tx);

	while (k_uptime_get() < end) {
		if (uart_poll_in(dev, &rx) == 0) {
			return rx == tx;
		}
	}
	return false;
}

ZTEST(uart_pm_lpf3, test_suspend_resume)
{
	const struct device *dev = DUT;
	enum pm_device_state st;

	zassert_true(device_is_ready(dev), "UART not ready");

	zassert_true(loopback_byte(dev, 0xA5), "loopback failed before suspend");

	zassert_ok(pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND), "suspend failed");
	zassert_ok(pm_device_state_get(dev, &st), "state_get failed");
	zassert_equal(st, PM_DEVICE_STATE_SUSPENDED, "state != SUSPENDED");

	zassert_ok(pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME), "resume failed");
	zassert_ok(pm_device_state_get(dev, &st), "state_get failed");
	zassert_equal(st, PM_DEVICE_STATE_ACTIVE, "state != ACTIVE");

	zassert_true(loopback_byte(dev, 0x5A), "loopback failed after resume (config lost?)");

	/* Multiple cycles: catch any state that leaks/accumulates across suspends. */
	for (int i = 0; i < 8; i++) {
		zassert_ok(pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND),
			   "suspend failed (cycle %d)", i);
		zassert_ok(pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME),
			   "resume failed (cycle %d)", i);
		zassert_true(loopback_byte(dev, (uint8_t)(0x10 + i)),
			     "loopback failed after resume (cycle %d)", i);
	}

	printk("uart_pm_lpf3: suspend/resume + loopback OK across 9 cycles\n");
}

ZTEST_SUITE(uart_pm_lpf3, NULL, NULL, NULL, NULL, NULL);
