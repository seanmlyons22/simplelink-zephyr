/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Regression tests for the TI LPF3 counter drivers:
 * - counter_cc23x0_rtc: alarm interrupt routing (EVTSVT CPUIRQnSEL must match
 *   the DT interrupt line), 8 us tick unit consistency, 64-bit value read.
 * - counter_cc23x0_lgpt / counter_cc27xx_lgpt: clock gate selection by base
 *   address (regression: only LGPT3 is enabled in the board overlay, so a
 *   driver using the DT instance number would clock the wrong LGPT and hang
 *   or fault on the first register access).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

static K_SEM_DEFINE(alarm_sem, 0, 1);
static volatile uint32_t alarm_ticks_at_cb;

static void alarm_cb(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(user_data);

	alarm_ticks_at_cb = ticks;
	k_sem_give(&alarm_sem);
}

#if DT_HAS_COMPAT_STATUS_OKAY(ti_cc23x0_rtc)

#define RTC_DEV DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(ti_cc23x0_rtc))

/* TIME8U resolution is 8 us per tick -> 125 kHz */
#define RTC_EXPECTED_FREQ 125000U
/* 50 ms in RTC ticks */
#define RTC_ALARM_TICKS   (RTC_EXPECTED_FREQ / 20U)

ZTEST(counter_lpf3_rtc, test_freq_matches_tick_unit)
{
	const struct device *dev = RTC_DEV;
	uint32_t freq = counter_get_frequency(dev);
	uint32_t v1, v2;

	zassert_equal(freq, RTC_EXPECTED_FREQ,
		      "RTC freq %u does not match the 8 us tick unit of get_value()", freq);

	zassert_ok(counter_get_value(dev, &v1));
	k_sleep(K_MSEC(100));
	zassert_ok(counter_get_value(dev, &v2));

	uint32_t delta = v2 - v1;

	/* 100 ms = 12500 ticks at 125 kHz, allow +-25% (scheduling slack) */
	zassert_true(delta > 9000 && delta < 16000,
		     "counter did not advance at get_frequency() rate: delta=%u", delta);
}

ZTEST(counter_lpf3_rtc, test_get_value_64_consistent)
{
	const struct device *dev = RTC_DEV;
	uint32_t v32;
	uint64_t v64;

	zassert_ok(counter_get_value(dev, &v32));
	zassert_ok(counter_get_value_64(dev, &v64));

	/*
	 * The 64-bit value contains the raw RTC counter, whose bits [34:3]
	 * are the 32-bit TIME8U value returned by counter_get_value().
	 */
	uint64_t diff = (v64 >> 3) - v32;

	zassert_true(diff < 16, "64-bit value inconsistent with 32-bit value (diff=%llu)", diff);

	/* Must be monotonic */
	uint64_t v64_2;

	k_sleep(K_MSEC(10));
	zassert_ok(counter_get_value_64(dev, &v64_2));
	zassert_true(v64_2 > v64, "64-bit counter not monotonic");
}

ZTEST(counter_lpf3_rtc, test_alarm_relative)
{
	const struct device *dev = RTC_DEV;
	const struct counter_alarm_cfg cfg = {
		.callback = alarm_cb,
		.ticks = RTC_ALARM_TICKS,
	};
	int64_t start = k_uptime_get();

	k_sem_reset(&alarm_sem);
	zassert_ok(counter_set_channel_alarm(dev, 0, &cfg));

	zassert_ok(k_sem_take(&alarm_sem, K_MSEC(500)), "RTC alarm interrupt never fired");

	int64_t elapsed = k_uptime_get() - start;

	zassert_true(elapsed >= 40 && elapsed <= 150,
		     "alarm fired after %lld ms, expected ~50 ms", elapsed);
}

ZTEST(counter_lpf3_rtc, test_alarm_absolute)
{
	const struct device *dev = RTC_DEV;
	uint32_t now;

	zassert_ok(counter_get_value(dev, &now));

	const struct counter_alarm_cfg cfg = {
		.callback = alarm_cb,
		.ticks = now + RTC_ALARM_TICKS,
		.flags = COUNTER_ALARM_CFG_ABSOLUTE,
	};

	k_sem_reset(&alarm_sem);
	zassert_ok(counter_set_channel_alarm(dev, 0, &cfg));
	zassert_ok(k_sem_take(&alarm_sem, K_MSEC(500)), "absolute RTC alarm never fired");
}

ZTEST(counter_lpf3_rtc, test_alarm_cancel)
{
	const struct device *dev = RTC_DEV;
	const struct counter_alarm_cfg cfg = {
		.callback = alarm_cb,
		.ticks = RTC_ALARM_TICKS,
	};

	k_sem_reset(&alarm_sem);
	zassert_ok(counter_set_channel_alarm(dev, 0, &cfg));
	zassert_ok(counter_cancel_channel_alarm(dev, 0));

	zassert_equal(k_sem_take(&alarm_sem, K_MSEC(120)), -EAGAIN,
		      "cancelled alarm still fired");
}

ZTEST(counter_lpf3_rtc, test_invalid_channel)
{
	const struct counter_alarm_cfg cfg = {
		.callback = alarm_cb,
		.ticks = RTC_ALARM_TICKS,
	};

	zassert_equal(counter_set_channel_alarm(RTC_DEV, 1, &cfg), -ENOTSUP);
}

ZTEST_SUITE(counter_lpf3_rtc, NULL, NULL, NULL, NULL, NULL);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(ti_cc23x0_rtc) */

#if DT_HAS_COMPAT_STATUS_OKAY(ti_cc23x0_lgpt)
#define LGPT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_cc23x0_lgpt)
#elif DT_HAS_COMPAT_STATUS_OKAY(ti_cc27xx_lgpt)
#define LGPT_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_cc27xx_lgpt)
#endif

#ifdef LGPT_NODE

#define LGPT_DEV DEVICE_DT_GET(LGPT_NODE)

ZTEST(counter_lpf3_lgpt, test_value_advances_when_started)
{
	const struct device *dev = LGPT_DEV;
	uint32_t freq = counter_get_frequency(dev);
	uint32_t top = counter_get_top_value(dev);
	uint32_t v1, v2;

	zassert_true(freq > 0, "invalid LGPT frequency");
	zassert_true(top > 0, "invalid LGPT top value");

	/* Wait for ~top/8 worth of ticks so the counter cannot wrap */
	uint32_t wait_us = CLAMP((uint64_t)(top / 8U) * USEC_PER_SEC / freq, 100U, 1000U);
	uint32_t expected = (uint64_t)freq * wait_us / USEC_PER_SEC;

	zassert_ok(counter_start(dev));
	zassert_ok(counter_get_value(dev, &v1));
	k_busy_wait(wait_us);
	zassert_ok(counter_get_value(dev, &v2));

	uint32_t delta = (v2 >= v1) ? (v2 - v1) : (top - v1 + v2 + 1);

	zassert_true(delta > expected / 2 && delta < expected * 2,
		     "LGPT did not tick at expected rate: delta=%u expected=%u (is the "
		     "right CLKCTL gate enabled?)", delta, expected);

	zassert_ok(counter_stop(dev));
}

ZTEST(counter_lpf3_lgpt, test_stops_counting_when_stopped)
{
	const struct device *dev = LGPT_DEV;
	uint32_t v1, v2;

	zassert_ok(counter_start(dev));
	k_busy_wait(100);
	zassert_ok(counter_stop(dev));

	zassert_ok(counter_get_value(dev, &v1));
	k_busy_wait(1000);
	zassert_ok(counter_get_value(dev, &v2));

	zassert_equal(v1, v2, "counter still running after stop (%u -> %u)", v1, v2);
}

ZTEST(counter_lpf3_lgpt, test_alarm_fires)
{
	const struct device *dev = LGPT_DEV;
	uint32_t freq = counter_get_frequency(dev);
	uint32_t top = counter_get_top_value(dev);
	const struct counter_alarm_cfg cfg = {
		.callback = alarm_cb,
		/* ~10 ms, clamped so the compare value fits below the top value */
		.ticks = MIN(freq / 100U, top / 2U),
	};

	k_sem_reset(&alarm_sem);
	zassert_ok(counter_start(dev));
	zassert_ok(counter_set_channel_alarm(dev, 0, &cfg));

	zassert_ok(k_sem_take(&alarm_sem, K_MSEC(200)), "LGPT alarm interrupt never fired");

	zassert_ok(counter_cancel_channel_alarm(dev, 0));
	zassert_ok(counter_stop(dev));
}

ZTEST_SUITE(counter_lpf3_lgpt, NULL, NULL, NULL, NULL, NULL);

#endif /* LGPT_NODE */
