/*
 * Copyright (c) 2026 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/ztest.h>

/*
 * Override the weak RCL wrapper symbol to simulate a failed noise capture:
 * leave the (driver-cleared) buffer empty and report success. The RCL
 * wrapper's completion status is unreliable for ADC-noise captures
 * (SDK bug-reports/RCL-core.md), so the driver must not trust it; it must
 * reject the capture based on the empty buffer content.
 *
 * The health tests (RCT/APT) are disabled in prj.conf so this exercises the
 * driver's own all-zeros content guard, which is the only gate left when the
 * health tests are off. Regression: without that guard an empty/failed
 * capture is hashed into the entropy pool and the device comes up seeded with
 * near-zero entropy.
 */
int_fast16_t RCL_AdcNoise_get_samples_blocking(uint32_t *buffer, uint32_t numWords)
{
	memset(buffer, 0, numWords * sizeof(uint32_t));
	return 0; /* "success" status with no noise produced */
}

ZTEST(entropy_lpf3_rng_init, test_init_rejects_empty_capture)
{
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));

	zassert_false(device_is_ready(dev),
		      "entropy device ready despite empty noise capture");
}

ZTEST_SUITE(entropy_lpf3_rng_init, NULL, NULL, NULL, NULL, NULL);
