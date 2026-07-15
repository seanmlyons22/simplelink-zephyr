/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Regression test: the RTC counter alarm interrupt must be delivered on the
 * CPU interrupt line assigned in the devicetree (CPUIRQ1), not on CPUIRQ3,
 * which the TI Power driver dedicates to the CKM oscillator interrupt.
 * With the event muxed to the wrong line the alarm callback never fires.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/counter.h>

#define ALARM_PERIOD_US 100000U

static K_SEM_DEFINE(alarm_sem, 0, 1);

static void alarm_cb(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(ticks);
	ARG_UNUSED(user_data);

	k_sem_give(&alarm_sem);
}

ZTEST(counter_cc23x0_alarm, test_alarm_callback_fires)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(rtc0));
	struct counter_alarm_cfg cfg = {
		.callback = alarm_cb,
		.ticks = counter_us_to_ticks(dev, ALARM_PERIOD_US),
		.user_data = NULL,
		.flags = 0,
	};

	zassert_true(device_is_ready(dev), "RTC counter not ready");
	zassert_ok(counter_start(dev));
	zassert_ok(counter_set_channel_alarm(dev, 0, &cfg));

	zassert_ok(k_sem_take(&alarm_sem, K_MSEC(1000)),
		   "alarm callback did not fire (RTC event muxed to wrong CPU IRQ line?)");

	zassert_ok(counter_cancel_channel_alarm(dev, 0));
}

ZTEST_SUITE(counter_cc23x0_alarm, NULL, NULL, NULL, NULL, NULL);
