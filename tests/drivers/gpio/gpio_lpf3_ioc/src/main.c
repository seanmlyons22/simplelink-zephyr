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

#include <driverlib/gpio.h>
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

static void test_before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_true(gpio_is_ready_dt(&out));
	zassert_true(gpio_is_ready_dt(&in));

	zassert_ok(gpio_pin_configure_dt(&out, GPIO_OUTPUT_LOW));
	zassert_ok(gpio_pin_configure_dt(&in, GPIO_INPUT));

	/* gpio_manage_callback tolerates re-adding the same node */
	gpio_init_callback(&cb_data, cb_handler, BIT(in.pin));
	zassert_ok(gpio_add_callback(in.port, &cb_data));
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

/*
 * pin_interrupt_configure() must not touch PORTCFG: the API is documented for
 * pins routed to other modules (I2C/SPI/UART) and the pinmux is owned by
 * pinctrl. Plant a non-GPIO mux value and check it survives.
 */
ZTEST(gpio_lpf3_ioc, test_int_configure_preserves_portcfg)
{
	uint32_t saved = IOC_REG(in.pin);

	IOC_REG(in.pin) = (saved & ~IOC_IOC0_PORTCFG_M) | IOC_IOC0_PORTCFG_ANA;

	zassert_ok(gpio_pin_interrupt_configure_dt(&in, GPIO_INT_EDGE_RISING));

	uint32_t portcfg = IOC_REG(in.pin) & IOC_IOC0_PORTCFG_M;

	/* Restore before asserting so a failure doesn't leave the pin on ANA */
	zassert_ok(gpio_pin_interrupt_configure_dt(&in, GPIO_INT_DISABLE));
	IOC_REG(in.pin) = saved;

	zassert_equal(portcfg, IOC_IOC0_PORTCFG_ANA,
		      "pin_interrupt_configure rewrote PORTCFG (pinmux)");
}

/*
 * GPIO_INT_WAKEUP at configure time arms wake-from-shutdown (WUCFGSD), which
 * is level-based: it must follow the pin's active level, not always wake-low.
 */
ZTEST(gpio_lpf3_ioc, test_shutdown_wake_polarity)
{
	zassert_ok(gpio_pin_configure_dt(&in, GPIO_INPUT | GPIO_INT_WAKEUP));
	zassert_equal(IOC_REG(in.pin) & IOC_IOC0_WUCFGSD_M, IOC_IOC0_WUCFGSD_WAKE_HIGH,
		      "active-high pin must wake on high level");

	zassert_ok(gpio_pin_configure(in.port, in.pin,
				      GPIO_INPUT | GPIO_ACTIVE_LOW | GPIO_INT_WAKEUP));
	zassert_equal(IOC_REG(in.pin) & IOC_IOC0_WUCFGSD_M, IOC_IOC0_WUCFGSD_WAKE_LOW,
		      "active-low pin must wake on low level");

	gpio_flags_t flags;

	zassert_ok(gpio_pin_get_config(in.port, in.pin, &flags));
	zassert_true(flags & GPIO_INT_WAKEUP, "get_config must report GPIO_INT_WAKEUP");

	zassert_ok(gpio_pin_configure_dt(&in, GPIO_INPUT));
}

/*
 * ngpios (and thus port_pin_mask) must not claim pins beyond the DIOs that
 * exist on the device (cc23x0: DIO0-25, cc27xx/cc27xxx10: DIO0-30). A larger
 * value lets nonexistent pins pass validation, so pin_configure writes land
 * in reserved IOC space.
 */
ZTEST(gpio_lpf3_ioc, test_port_pin_mask_matches_hw)
{
	const struct gpio_driver_config *cfg = in.port->config;

	zassert_equal(cfg->port_pin_mask & ~(uint32_t)GPIO_DIO_ALL_MASK, 0,
		      "port_pin_mask allows pins beyond the device's DIOs");
}

ZTEST_SUITE(gpio_lpf3_ioc, NULL, NULL, test_before, test_after, NULL);
