/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Regression test for the LPF3 ClockP system timer announce baseline.
 *
 * With an announce baseline that is not aligned to the kernel tick grid,
 * every 1-tick timeout fires one interrupt early with sys_clock_announce(0)
 * and only completes on the following tick, so each k_sleep(K_TICKS(1))
 * takes ~2 ticks of wall time instead of ~1.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define LOOPS 100

ZTEST(lpf3_tick_timing, test_one_tick_sleep_duration)
{
	uint32_t cycles_per_tick =
		sys_clock_hw_cycles_per_sec() / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	uint32_t t0;
	uint32_t elapsed;

	/* Settle onto a tick boundary */
	k_sleep(K_TICKS(1));

	t0 = k_cycle_get_32();

	for (int i = 0; i < LOOPS; i++) {
		k_sleep(K_TICKS(1));
	}

	elapsed = k_cycle_get_32() - t0;

	TC_PRINT("%d one-tick sleeps took %u cycles (%u per tick)\n",
		 LOOPS, elapsed, cycles_per_tick);

	/* Each 1-tick sleep must complete on the next tick boundary. Allow
	 * 50% margin for ISR/scheduling overhead; the unaligned-baseline bug
	 * doubles the duration (~200%), well past this threshold.
	 */
	zassert_true(elapsed < (LOOPS * 3U / 2U) * cycles_per_tick,
		     "1-tick sleeps too slow: %u cycles for %d sleeps",
		     elapsed, LOOPS);
	zassert_true(elapsed >= (LOOPS - 1U) * cycles_per_tick,
		     "1-tick sleeps impossibly fast: %u cycles", elapsed);
}

ZTEST_SUITE(lpf3_tick_timing, NULL, NULL, NULL, NULL, NULL);
