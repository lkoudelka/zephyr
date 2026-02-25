/*
 * Copyright 2025 CodeWrights GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_vrefbuf

#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>
#include <stm32_ll_system.h>

LOG_MODULE_REGISTER(regulator_stm32_vrefbuf, CONFIG_REGULATOR_LOG_LEVEL);

struct regulator_stm32_vrefbuf_voltage {
	uint32_t vrs; /* LL_VREFBUF_VOLTAGE_SCALE<X> */
	int32_t uv;   /* VREFBUF output in uV with this scale */
};

struct regulator_stm32_vrefbuf_data {
	struct regulator_common_data common;
};

struct regulator_stm32_vrefbuf_config {
	struct regulator_common_config common;

	
	const struct stm32_pclken pclken[1];
	const struct reset_dt_spec reset;

	const struct regulator_stm32_vrefbuf_voltage *ref_voltages;
	/*
	 * Some STM32 series gate VREFBUF behind RCC clock/reset, described by
	 * DT properties 'clocks' and 'resets'. Others (e.g. STM32WB) have no
	 * such gates; in that case those DT properties are absent.
	 *
	 * We use a single flag because, for STM32 series where VREFBUF has a
	 * clock gate, it also has a reset line in Zephyr DTS.
	 */
	bool has_clock;
	bool vrefp_output_enable;
	uint8_t ref_voltage_count;
};

static int regulator_stm32_vrefbuf_enable(const struct device *dev)
{
	ARG_UNUSED(dev);

	LL_VREFBUF_Enable();

	return 0;
}

static int regulator_stm32_vrefbuf_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	LL_VREFBUF_Disable();

	return 0;
}

static int regulator_stm32_vrefbuf_list_voltage(const struct device *dev, unsigned int idx,
						int32_t *volt_uv)
{
	const struct regulator_stm32_vrefbuf_config *config = dev->config;

	if (idx >= config->ref_voltage_count) {
		return -EINVAL;
	}

	*volt_uv = config->ref_voltages[idx].uv;

	return 0;
}

static unsigned int regulator_stm32_vrefbuf_count_voltages(const struct device *dev)
{
	const struct regulator_stm32_vrefbuf_config *config = dev->config;

	return config->ref_voltage_count;
}

static int regulator_stm32_vrefbuf_set_voltage(const struct device *dev, int32_t min_uv,
					       int32_t max_uv)
{
	const struct regulator_stm32_vrefbuf_config *config = dev->config;
	uint32_t best_vrs = 0;
	int32_t best_voltage = INT32_MAX;

	/* Search among supported voltages for the closest one at least equal
	 * to minimum requested by caller. Note that all configurations must
	 * be checked as the range order is not consistent across series
	 * (i.e., whether VREFBUF0 < VREFBUF1 or VREFBUF0 > VREFBUF1).
	 */
	for (uint8_t i = 0; i < config->ref_voltage_count; i++) {
		if (IN_RANGE(config->ref_voltages[i].uv, min_uv, max_uv) &&
		    config->ref_voltages[i].uv < best_voltage) {
			best_voltage = config->ref_voltages[i].uv;
			best_vrs = config->ref_voltages[i].vrs;
		}
	}

	if (best_voltage == INT32_MAX) {
		return -EINVAL;
	}

	LL_VREFBUF_SetVoltageScaling(best_vrs);

	return 0;
}

static int regulator_stm32_vrefbuf_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_stm32_vrefbuf_config *config = dev->config;

	uint32_t vrs = LL_VREFBUF_GetVoltageScaling();

	for (uint8_t i = 0; i < config->ref_voltage_count; i++) {
		if (config->ref_voltages[i].vrs == vrs) {
			*volt_uv = config->ref_voltages[i].uv;
			return 0;
		}
	}

	return -EIO;
}

static int regulator_stm32_vrefbuf_init(const struct device *dev)
{
	const struct regulator_stm32_vrefbuf_config *config = dev->config;

	regulator_common_data_init(dev);

	/*
	 * Optional clock/reset support:
	 * - If DT has a 'clocks' property, enable the peripheral clock and
	 *   deassert reset.
	 * - If not, do nothing (e.g. STM32WB: VREFBUF is always clocked).
	 */
	if (config->has_clock) {
		const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

		if (clock_control_on(clk, (clock_control_subsys_t)&config->pclken[0]) != 0) {
			LOG_ERR("Could not enable clock");
			return -EIO;
		}

		if (!device_is_ready(config->reset.dev)) {
			LOG_ERR("Reset controller not ready");
			return -ENODEV;
		}

		if (reset_line_deassert_dt(&config->reset) != 0) {
			LOG_ERR("Could not deassert reset line");
			return -EIO;
		}
	}

	if (config->vrefp_output_enable) {
		LL_VREFBUF_DisableHIZ();
	} else {
		LL_VREFBUF_EnableHIZ();
	}

	return regulator_common_init(dev, false);
}

static DEVICE_API(regulator, api) = {
	.enable = regulator_stm32_vrefbuf_enable,
	.disable = regulator_stm32_vrefbuf_disable,
	.count_voltages = regulator_stm32_vrefbuf_count_voltages,
	.list_voltage = regulator_stm32_vrefbuf_list_voltage,
	.set_voltage = regulator_stm32_vrefbuf_set_voltage,
	.get_voltage = regulator_stm32_vrefbuf_get_voltage,
};

#define VREFBUF_VOLTAGE_ELEM(node_id, prop, idx)                                                   \
	{                                                                                          \
		.vrs = CONCAT(LL_VREFBUF_VOLTAGE_SCALE, idx),                                      \
		.uv = DT_PROP_BY_IDX(node_id, prop, idx)                                           \
	},

/*
 * If the instance provides 'clocks', we assume it also provides 'resets'
 * (true for STM32 series where VREFBUF is gated in Zephyr DTS).
 *
 * Keep .pclken as an array [1] to match the STM32_DT_INST_CLOCKS(inst)
 * initializer style used across Zephyr STM32 drivers.
 */
#define VREFBUF_CLOCK_INIT(inst)                                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, clocks),                                            \
		    (.has_clock = true,                                                             \
		     .pclken = STM32_DT_INST_CLOCKS(inst),                                          \
		     .reset = RESET_DT_SPEC_GET(DT_DRV_INST(inst)),),                               \
		    (.has_clock = false,                                                           \
		     .pclken = {0},                                                                \
		     .reset = (struct reset_dt_spec){0},))

#define REGULATOR_STM32_VREFBUF_DEFINE(inst)                                                       \
	static struct regulator_stm32_vrefbuf_data data_##inst;                                    \
                                                                                                   \
	static const struct regulator_stm32_vrefbuf_voltage ref_voltages_##inst[] = {              \
		DT_FOREACH_PROP_ELEM(DT_DRV_INST(inst), ref_voltages, VREFBUF_VOLTAGE_ELEM)        \
	};                                                                                         \
                                                                                                   \
	static const struct regulator_stm32_vrefbuf_config config_##inst = {                       \
		.common = REGULATOR_DT_INST_COMMON_CONFIG_INIT(inst),                              \
		VREFBUF_CLOCK_INIT(inst)                                                          \
		.vrefp_output_enable = DT_INST_PROP(inst, vrefp_output_enable),                    \
		.ref_voltages = ref_voltages_##inst,                                               \
		.ref_voltage_count = ARRAY_SIZE(ref_voltages_##inst),                              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, regulator_stm32_vrefbuf_init, NULL, &data_##inst,              \
			      &config_##inst, POST_KERNEL,                                         \
			      CONFIG_REGULATOR_STM32_VREFBUF_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_STM32_VREFBUF_DEFINE)