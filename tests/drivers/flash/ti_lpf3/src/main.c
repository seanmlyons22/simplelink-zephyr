/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Regression tests for the TI LPF3 (CC23x0/CC27xx) SoC flash drivers:
 * range validation of erase/write/read.
 *
 * All actual flash traffic stays inside storage_partition. The CCFG
 * sector (0x4E020000, flash1) is deliberately never addressed: on a
 * driver without erase range validation, HapiFlashSectorErase() accepts
 * the CCFG address and erasing it can brick the device. Out-of-range
 * offsets used here (main flash size, negative) are rejected by the
 * flash HAPI itself (FAPI_STATUS_ADDRESS_ERROR), so they are safe to
 * issue even on an unfixed driver.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/ztest.h>

#define FLASH_NODE   DT_CHOSEN(zephyr_flash) /* flash0: main flash */
#define FLASH_SIZE_B DT_REG_SIZE(FLASH_NODE)
#define ERASE_SIZE   DT_PROP(FLASH_NODE, erase_block_size)
#define WRITE_SIZE   DT_PROP(FLASH_NODE, write_block_size)

#define STORAGE_NODE DT_NODELABEL(storage_partition)
#define STORAGE_OFF  DT_REG_ADDR(STORAGE_NODE)

static const struct device *flash_dev =
	DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(STORAGE_NODE));

ZTEST(flash_ti_lpf3, test_erase_out_of_range)
{
	/* One sector starting right past the end of main flash */
	zassert_equal(flash_erase(flash_dev, FLASH_SIZE_B, ERASE_SIZE), -EINVAL,
		      "erase beyond main flash must return -EINVAL");

	/* Last valid sector, but length runs past the end */
	zassert_equal(flash_erase(flash_dev, FLASH_SIZE_B - ERASE_SIZE, 2 * ERASE_SIZE), -EINVAL,
		      "erase running past main flash must return -EINVAL");

	/* Negative offset (sector-aligned, so it passes the modulo check) */
	zassert_equal(flash_erase(flash_dev, -(off_t)ERASE_SIZE, ERASE_SIZE), -EINVAL,
		      "erase at negative offset must return -EINVAL");
}

ZTEST(flash_ti_lpf3, test_write_read_out_of_range)
{
	uint8_t buf[2 * WRITE_SIZE];

	zassert_equal(flash_write(flash_dev, FLASH_SIZE_B, buf, WRITE_SIZE), -EINVAL,
		      "write beyond main flash must return -EINVAL");
	zassert_equal(flash_write(flash_dev, FLASH_SIZE_B - WRITE_SIZE, buf, sizeof(buf)), -EINVAL,
		      "write running past main flash must return -EINVAL");

	zassert_equal(flash_read(flash_dev, FLASH_SIZE_B, buf, WRITE_SIZE), -EINVAL,
		      "read beyond main flash must return -EINVAL");
	zassert_equal(flash_read(flash_dev, FLASH_SIZE_B - WRITE_SIZE, buf, sizeof(buf)), -EINVAL,
		      "read running past main flash must return -EINVAL");
}

/* Happy path: the checks above must not break normal operation */
ZTEST(flash_ti_lpf3, test_erase_write_read_roundtrip)
{
	static uint8_t wr[2 * WRITE_SIZE]; /* static: source must not be in flash */
	uint8_t rd[2 * WRITE_SIZE];

	for (size_t i = 0; i < sizeof(wr); i++) {
		wr[i] = (uint8_t)(i ^ 0xa5);
	}

	zassert_ok(flash_erase(flash_dev, STORAGE_OFF, ERASE_SIZE), "erase failed");
	zassert_ok(flash_write(flash_dev, STORAGE_OFF, wr, sizeof(wr)), "write failed");
	zassert_ok(flash_read(flash_dev, STORAGE_OFF, rd, sizeof(rd)), "read failed");
	zassert_mem_equal(wr, rd, sizeof(wr), "readback mismatch");

	/* Rest of the sector still erased */
	zassert_ok(flash_read(flash_dev, STORAGE_OFF + sizeof(wr), rd, sizeof(rd)));
	for (size_t i = 0; i < sizeof(rd); i++) {
		zassert_equal(rd[i], 0xff, "byte %zu not erased", i);
	}
}

ZTEST_SUITE(flash_ti_lpf3, NULL, NULL, NULL, NULL, NULL);
