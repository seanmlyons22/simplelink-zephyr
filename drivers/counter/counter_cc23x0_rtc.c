/*
 * Copyright (c) 2024 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_cc23x0_rtc

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include <inc/hw_rtc.h>
#include <inc/hw_types.h>
#include <inc/hw_evtsvt.h>
#include <inc/hw_memmap.h>

LOG_MODULE_REGISTER(cc23x0_counter_rtc, CONFIG_COUNTER_LOG_LEVEL);

/*
 * The RTC counter value exposed by this driver is the TIME8U register,
 * i.e. bits [34:3] of the 67-bit RTC counter, with a resolution of 8 us
 * per tick (TRM chapter 12, "RTC"). Hence the counter frequency seen
 * through the Zephyr counter API is 125 kHz.
 */
#define RTC_CC23X0_FREQ_HZ 125000U

/*
 * The RTC event must be routed to the CPU interrupt line this driver is
 * connected to (from devicetree). CPUIRQnSEL registers are contiguous,
 * 4 bytes apart, starting at CPUIRQ0SEL (TRM table 4-18), and all reset
 * to 0 (no publisher selected).
 */
#define RTC_CC23X0_CPUIRQ_SEL_REG                                                                  \
	(EVTSVT_BASE + EVTSVT_O_CPUIRQ0SEL + sizeof(uint32_t) * DT_INST_IRQN(0))

static void counter_cc23x0_isr(const struct device *dev);

struct counter_cc23x0_config {
	struct counter_config_info counter_info;
	uint32_t base;
};

struct counter_cc23x0_data {
	struct counter_alarm_cfg alarm_cfg0;
	uint32_t ticks;
	uint32_t clk_src_freq;
};

static int counter_cc23x0_get_value(const struct device *dev, uint32_t *ticks)
{
	/* Resolution is 8us and max timeout ~9.5h */
	const struct counter_cc23x0_config *config = dev->config;

	*ticks = HWREG(config->base + RTC_O_TIME8U);

	return 0;
}

static int counter_cc23x0_get_value_64(const struct device *dev, uint64_t *ticks)
{
	const struct counter_cc23x0_config *config = dev->config;

	/*
	 * RTC counter register is 67 bits, only part of the bits are accessible.
	 * They are split in two partially overlapping registers:
	 * TIME524M [50:19]
	 * TIME8U   [34:3]
	 *
	 * The values must be widened to 64 bits before shifting, otherwise
	 * the TIME524M contribution (bits [50:35] of the counter) and the
	 * top 3 bits of TIME8U are shifted out of a 32-bit intermediate.
	 * TIME8U is read between the two TIME524M reads to detect a carry
	 * into TIME524M and retry, so both reads belong to the same instant.
	 */
	uint32_t time524m = HWREG(config->base + RTC_O_TIME524M);
	uint32_t time8u = HWREG(config->base + RTC_O_TIME8U);

	if (HWREG(config->base + RTC_O_TIME524M) != time524m) {
		time524m = HWREG(config->base + RTC_O_TIME524M);
		time8u = HWREG(config->base + RTC_O_TIME8U);
	}

	*ticks = (((uint64_t)time524m << 19) & 0xFFFFFFF800000000ULL) |
		 ((uint64_t)time8u << 3);

	return 0;
}

static void counter_cc23x0_isr(const struct device *dev)
{
	const struct counter_cc23x0_config *config = dev->config;
	struct counter_cc23x0_data *data = dev->data;

	/* Clear and mask channel 0 interrupt (alarms are one-shot) */
	HWREG(config->base + RTC_O_ICLR) = 0x1;
	HWREG(config->base + RTC_O_IMCLR) = 0x1;

	uint32_t now = HWREG(config->base + RTC_O_TIME8U);
	counter_alarm_callback_t cb = data->alarm_cfg0.callback;

	if (cb) {
		data->alarm_cfg0.callback = NULL;
		cb(dev, 0, now, data->alarm_cfg0.user_data);
	}
}

static int counter_cc23x0_set_alarm(const struct device *dev, uint8_t chan_id,
				    const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_cc23x0_config *config = dev->config;
	struct counter_cc23x0_data *data = dev->data;
	uint32_t next_alarm;

	if (chan_id >= config->counter_info.channels) {
		return -ENOTSUP;
	}

	if (!alarm_cfg->ticks) {
		return -EINVAL;
	}

	/*
	 * Ticks are in units of the exposed counter value (TIME8U, 8 us per
	 * tick), so they can be written to the CH0 compare register as is.
	 */
	if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		next_alarm = alarm_cfg->ticks;
	} else {
		next_alarm = HWREG(config->base + RTC_O_TIME8U) + alarm_cfg->ticks;
	}

	data->alarm_cfg0.flags = alarm_cfg->flags;
	data->alarm_cfg0.ticks = alarm_cfg->ticks;
	data->alarm_cfg0.callback = alarm_cfg->callback;
	data->alarm_cfg0.user_data = alarm_cfg->user_data;

	HWREG(config->base + RTC_O_CH0CC8U) = next_alarm;
	HWREG(config->base + RTC_O_IMASK) = 0x1;
	HWREG(config->base + RTC_O_ARMSET) = 0x1;

	/*
	 * Route the RTC event to the CPU interrupt line used by this driver.
	 * The DT interrupt number matches the CPUIRQn line (CPUIRQ1 for the
	 * default DT), whose select register resets to 0 (no source).
	 */
	HWREG(RTC_CC23X0_CPUIRQ_SEL_REG) = EVTSVT_CPUIRQ0SEL_PUBID_AON_RTC_COMB;

	return 0;
}

static int counter_cc23x0_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct counter_cc23x0_config *config = dev->config;
	struct counter_cc23x0_data *data = dev->data;

	if (chan_id >= config->counter_info.channels) {
		return -ENOTSUP;
	}

	/* Unset interrupt source */
	HWREG(RTC_CC23X0_CPUIRQ_SEL_REG) = 0x0;

	/* Unarm channel 0 (channel 1 is not managed by this driver) */
	HWREG(config->base + RTC_O_ARMCLR) = 0x1;

	data->alarm_cfg0.callback = NULL;

	return 0;
}

static int counter_cc23x0_set_top_value(const struct device *dev,
					const struct counter_top_cfg *cfg)
{
	return -ENOTSUP;
}

static uint32_t counter_cc23x0_get_pending_int(const struct device *dev)
{
	const struct counter_cc23x0_config *config = dev->config;

	/* Check interrupt and mask */
	return (HWREG(config->base + RTC_O_RIS) & HWREG(config->base + RTC_O_MIS)) ? 1 : 0;
}

#ifdef CONFIG_PM_DEVICE

static int rtc_cc23x0_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct counter_cc23x0_data *data = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		return 0;
	case PM_DEVICE_ACTION_RESUME:
		/*
		 * The RTC itself keeps running in standby, but the event
		 * fabric routing is lost when the SVT domain powers down.
		 * Restore it if an alarm is still armed.
		 */
		if (data->alarm_cfg0.callback) {
			HWREG(RTC_CC23X0_CPUIRQ_SEL_REG) = EVTSVT_CPUIRQ0SEL_PUBID_AON_RTC_COMB;
		}
		return 0;
	default:
		return -ENOTSUP;
	}
}

#endif /* CONFIG_PM_DEVICE */

static uint32_t counter_cc23x0_get_top_value(const struct device *dev)
{
	const struct counter_cc23x0_config *config = dev->config;

	/* Free-running counter, top value is fixed */
	return config->counter_info.max_top_value;
}

static uint32_t counter_cc23x0_get_freq(const struct device *dev)
{
	ARG_UNUSED(dev);

	/*
	 * The counter value exposed by get_value() is TIME8U, which has a
	 * resolution of 8 us per tick (TRM chapter 12), i.e. 125 kHz. This
	 * must match the unit used for alarm ticks and get_value().
	 */
	return RTC_CC23X0_FREQ_HZ;
}

static int counter_cc23x0_start(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* RTC timer runs after power-on reset */

	return 0;
}

static int counter_cc23x0_stop(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Any reset/sleep mode, except for POR, will not stop or reset the RTC timer */

	return 0;
}

static int counter_cc23x0_init(const struct device *dev)
{
	const struct counter_cc23x0_config *config = dev->config;

	/* Clear interrupt Mask */
	HWREG(config->base + RTC_O_IMCLR) = 0x3;

	/* Clear Interrupt */
	HWREG(config->base + RTC_O_ICLR) = 0x3;

	/* Clear Armed */
	HWREG(config->base + RTC_O_ARMCLR) = 0x3;

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    counter_cc23x0_isr,
		    DEVICE_DT_INST_GET(0),
		    0);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static const struct counter_driver_api rtc_cc23x0_api = {
	.start = counter_cc23x0_start,
	.stop = counter_cc23x0_stop,
	.get_value = counter_cc23x0_get_value,
	.get_value_64 = counter_cc23x0_get_value_64,
	.set_alarm = counter_cc23x0_set_alarm,
	.cancel_alarm = counter_cc23x0_cancel_alarm,
	.get_top_value = counter_cc23x0_get_top_value,
	.set_top_value = counter_cc23x0_set_top_value,
	.get_pending_int = counter_cc23x0_get_pending_int,
	.get_freq = counter_cc23x0_get_freq,
};

#define CC23X0_INIT(inst)									\
	PM_DEVICE_DT_INST_DEFINE(inst, rtc_cc23x0_pm_action);					\
												\
	static const struct counter_cc23x0_config cc23x0_config_##inst = {			\
	.counter_info = {									\
		.max_top_value = UINT32_MAX,							\
		.flags = COUNTER_CONFIG_INFO_COUNT_UP,						\
		.channels = 1,									\
	},											\
		.base = DT_INST_REG_ADDR(inst),							\
	};											\
												\
	static struct counter_cc23x0_data cc23x0_data_##inst;					\
												\
	DEVICE_DT_INST_DEFINE(0, &counter_cc23x0_init, PM_DEVICE_DT_INST_GET(inst),		\
			      &cc23x0_data_##inst, &cc23x0_config_##inst, POST_KERNEL,		\
			      CONFIG_COUNTER_INIT_PRIORITY, &rtc_cc23x0_api);

DT_INST_FOREACH_STATUS_OKAY(CC23X0_INIT)
