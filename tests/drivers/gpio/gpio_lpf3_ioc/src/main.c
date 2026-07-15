/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Regression tests for the TI LPF3 (CC23x0/CC27xx) GPIO driver IOC handling.
 * Requires the board loopback jumper between out-gpios and in-gpios.
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#include <inc/hw_ioc.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>

#define IOC_REG(pin) HWREG(IOC_BASE + IOC_O_IOC0 + sizeof(uint32_t) * (pin))

static const struct gpio_dt_spec out = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), out_gpios);
static const struct gpio_dt_spec in = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), in_gpios);

static struct gpio_callback cb_data;
static atomic_t cb_count;

static void cb_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	atomic_inc(&cb_count);
}

static void *suite_setup(void)
{
	zassert_true(gpio_is_ready_dt(&out));
	zassert_true(gpio_is_ready_dt(&in));

	gpio_init_callback(&cb_data, cb_handler, BIT(in.pin));
	zassert_ok(gpio_add_callback(in.port, &cb_data));

	return NULL;
}

static void test_before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_ok(gpio_pin_configure_dt(&out, GPIO_OUTPUT_LOW));
	zassert_ok(gpio_pin_configure_dt(&in, GPIO_INPUT));
	atomic_set(&cb_count, 0);
}

static void test_after(void *fixture)
{
	ARG_UNUSED(fixture);

	(void)gpio_pin_interrupt_configure_dt(&in, GPIO_INT_DISABLE);
}

/*
 * The GPIO_INT_WAKEUP trigger modifier is forwarded to the driver as part of
 * "trig" (see z_impl_gpio_pin_interrupt_configure). The driver must accept it:
 * edge interrupts always arm IOC.IOCn.WUENSB (standby wake) on this hardware.
 */
ZTEST(gpio_lpf3_ioc, test_int_wakeup_trig_accepted)
{
	zassert_ok(gpio_pin_interrupt_configure_dt(&in, GPIO_INT_EDGE_RISING | GPIO_INT_WAKEUP),
		   "GPIO_INT_WAKEUP trigger modifier rejected");

	zassert_ok(gpio_pin_set_raw(out.port, out.pin, 1));
	k_sleep(K_MSEC(5));
	zassert_equal(atomic_get(&cb_count), 1, "no interrupt after rising edge");
}

/*
 * gpio_pin_configure() must not disturb the interrupt trigger installed by
 * gpio_pin_interrupt_configure(): EDGEDET/WUENSB live in the same IOCn
 * register as the pin configuration.
 */
ZTEST(gpio_lpf3_ioc, test_pin_configure_preserves_int_trigger)
{
	zassert_ok(gpio_pin_interrupt_configure_dt(&in, GPIO_INT_EDGE_RISING));

	/* Reconfigure the pin (e.g. runtime pull change) after the trigger */
	zassert_ok(gpio_pin_configure_dt(&in, GPIO_INPUT | GPIO_PULL_DOWN));

	zassert_ok(gpio_pin_set_raw(out.port, out.pin, 1));
	k_sleep(K_MSEC(5));
	zassert_equal(atomic_get(&cb_count), 1,
		      "interrupt trigger lost after gpio_pin_configure()");
}

ZTEST_SUITE(gpio_lpf3_ioc, NULL, suite_setup, test_before, test_after, NULL);
