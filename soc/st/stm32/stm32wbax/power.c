/*
 * Copyright (c) 2022 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/pm/pm.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/arch/arm/cortex_m/fpu.h>
#include <kernel_arch_func.h>

#include <cmsis_core.h>

#include <stm32wbaxx_ll_bus.h>
#include <stm32wbaxx_ll_cortex.h>
#include <stm32wbaxx_ll_pwr.h>
#include <stm32wbaxx_ll_rcc.h>
#include <stm32wbaxx_ll_system.h>
#include <clock_control/clock_stm32_ll_common.h>

#ifdef CONFIG_BT_STM32WBA
#include "linklayer_plat.h"
#include "ll_sys.h"
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#if defined(CONFIG_PM_S2RAM)
SCB_Type backup_scb;

static void scb_suspend(SCB_Type *backup)
{
	backup->ICSR = SCB->ICSR;
	backup->VTOR = SCB->VTOR;
	backup->AIRCR = SCB->AIRCR;
	backup->SCR = SCB->SCR;
	backup->CCR = SCB->CCR;
	memcpy((uint32_t *) backup->SHPR, (uint32_t *)SCB->SHPR, sizeof(SCB->SHPR));
	backup->SHCSR = SCB->SHCSR;
	backup->CFSR = SCB->CFSR;
	backup->HFSR = SCB->HFSR;
	backup->DFSR = SCB->DFSR;
	backup->MMFAR = SCB->MMFAR;
	backup->BFAR = SCB->BFAR;
	backup->AFSR = SCB->AFSR;
	backup->CPACR = SCB->CPACR;
}
static void scb_resume(SCB_Type *backup)
{
	SCB->ICSR = backup->ICSR;
	SCB->VTOR = backup->VTOR;
	SCB->AIRCR = backup->AIRCR;
	SCB->SCR = backup->SCR;
	SCB->CCR = backup->CCR;
	memcpy((uint32_t *)SCB->SHPR, (uint32_t *)backup->SHPR, sizeof(SCB->SHPR));
	SCB->SHCSR = backup->SHCSR;
	SCB->CFSR = backup->CFSR;
	SCB->HFSR = backup->HFSR;
	SCB->DFSR = backup->DFSR;
	SCB->MMFAR = backup->MMFAR;
	SCB->BFAR = backup->BFAR;
	SCB->AFSR = backup->AFSR;
	SCB->CPACR = backup->CPACR;
}
#endif

void stm32_power_init(void);

static void set_mode_stop(uint8_t substate_id)
{

	LL_PWR_ClearFlag_STOP();
	LL_RCC_ClearResetFlags();

	/* Erratum 2.2.15:
	 * Disabling ICACHE is required before entering stop mode
	 */
	sys_cache_instr_disable();

	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_3);
	while (__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_3);
	MODIFY_REG(RAMCFG_SRAM1->CR, RAMCFG_CR_WSC, RAMCFG_WAITSTATE_1);
	MODIFY_REG(RAMCFG_SRAM2->CR, RAMCFG_CR_WSC, RAMCFG_WAITSTATE_1);

	/* Set SLEEPDEEP bit of Cortex System Control Register */
	LL_LPM_EnableDeepSleep();

	while (LL_PWR_IsActiveFlag_ACTVOS() == 0) {
	}

	switch (substate_id) {
	case 1: /* enter STOP0 mode */
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP0);
		break;
	case 2: /* enter STOP1 mode */
		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP1);
		break;
	default:
		LOG_DBG("Unsupported power state substate-id %u", substate_id);
		break;
	}
}

#if defined(CONFIG_PM_S2RAM)
static int suspend_to_ram(void)
{
	LL_LPM_EnableDeepSleep();

	while (LL_PWR_IsActiveFlag_ACTVOS() == 0) {
	}

	/* Select mode entry : WFE or WFI and enter the CPU selected mode */
	k_cpu_idle();

	return 0;
}

static void set_mode_suspend_to_ram(void)
{
	struct fpu_ctx_full fpu_state;

	/* Enable RTC wakeup
	 * This configures an internal pin that generates an event to wakeup the system
	 */
	LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN7);
	LL_PWR_SetWakeUpPinSignal3Selection(LL_PWR_WAKEUP_PIN7);

	/* Clear flags */
	LL_PWR_ClearFlag_SB();
	LL_PWR_ClearFlag_WU();
	LL_RCC_ClearResetFlags();

	sys_cache_instr_disable();

	/* Select standby mode */
	LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);

	/* Save FPU state and configuration */
	z_arm_save_fp_context(&fpu_state);
	scb_suspend(&backup_scb);
	/* Save context and enter Standby mode */
	arch_pm_s2ram_suspend(suspend_to_ram);

#if defined(CONFIG_ARM_MPU)
	z_arm_mpu_init();
	/* Configure static memory map. This will program MPU regions,
	 * to set up access permissions for fixed memory sections, such
	 * as Application Memory or No-Cacheable SRAM area.
	 *
	 * This function is invoked once, upon system initialization.
	 */
	z_arm_configure_static_mpu_regions();
#endif /* CONFIG_ARM_MPU */
	z_arm_fault_init();
	scb_resume(&backup_scb);
	z_arm_restore_fp_context(&fpu_state);

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_RAMCFG_CLK_ENABLE();

	/* Execution is restored at this point after wake up */
	/* Restore system clock as soon as we exit standby mode */
	stm32_clock_control_standby_exit();
}
#endif

/* Invoke Low Power/System Off specific Tasks */
void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		set_mode_stop(substate_id);

		/* Select mode entry : WFE or WFI and enter the CPU selected mode */
		k_cpu_idle();

		break;
#if defined(CONFIG_PM_S2RAM)
	case PM_STATE_SUSPEND_TO_RAM:
		set_mode_suspend_to_ram();
		break;
#endif
	default:
		LOG_DBG("Unsupported power state %u", state);
		return;
	}
}

/* Handle SOC specific activity after Low Power Mode Exit */
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	/* Init clock before all the post ops */
	if (LL_PWR_IsActiveFlag_STOP() || LL_PWR_IsActiveFlag_SB())
	{
		stm32_clock_control_init(NULL);
#ifdef CONFIG_BT_STM32WBA
		ll_sys_dp_slp_exit();
#endif
	} else {
		/* Apply waitsates for HSE32 configuration */
		__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
		while (__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_0);
		MODIFY_REG(RAMCFG_SRAM1->CR, RAMCFG_CR_WSC, RAMCFG_WAITSTATE_0);
		MODIFY_REG(RAMCFG_SRAM2->CR, RAMCFG_CR_WSC, RAMCFG_WAITSTATE_0);
	}

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		if (substate_id <= 2) {
			/* Erratum 2.2.15:
			 * Enable ICACHE when exiting stop mode
			 */
			sys_cache_instr_enable();

			LL_LPM_DisableSleepOnExit();
			LL_LPM_EnableSleep();
		} else {
			LOG_DBG("Unsupported power substate-id %u",
							substate_id);
		}
		break;
	case PM_STATE_SUSPEND_TO_RAM:
#if defined(CONFIG_PM_S2RAM)
		stm32wba_init();
		stm32_power_init();

		LL_LPM_DisableSleepOnExit();
		LL_LPM_EnableSleep();
#else
		LOG_DBG("Suspend to RAM needs CONFIG_PM_S2RAM to be enabled");
#endif
		break;
	case PM_STATE_STANDBY:
		__fallthrough;
	case PM_STATE_SUSPEND_TO_DISK:
		__fallthrough;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	/*
	 * System is now in active mode.
	 * Reenable interrupts which were disabled
	 * when OS started idling code.
	 */
	irq_unlock(0);
}

/* Initialize STM32 Power */
void stm32_power_init(void)
{

	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_PWR);

#ifdef CONFIG_DEBUG
	LL_DBGMCU_EnableDBGStopMode();
	LL_DBGMCU_EnableDBGStandbyMode();
	LL_DBGMCU_APB7_GRP1_FreezePeriph(LL_DBGMCU_APB7_GRP1_RTC_STOP);
	LL_DBGMCU_APB7_GRP1_FreezePeriph(LL_DBGMCU_APB7_GRP1_LPTIM1_STOP);
#else
	LL_DBGMCU_DisableDBGStandbyMode();
	LL_DBGMCU_DisableDBGStopMode();
#endif

	/* Enable SRAM full retention */
	LL_PWR_SetSRAM1SBRetention(LL_PWR_SRAM1_SB_FULL_RETENTION);
	LL_PWR_SetSRAM2SBRetention(LL_PWR_SRAM2_SB_FULL_RETENTION);

	/* Enable Radio RAM full retention */
	LL_PWR_SetRadioSBRetention(LL_PWR_RADIO_SB_FULL_RETENTION);

	/* Enabling  Ultra Low power mode */
	LL_PWR_EnableUltraLowPowerMode();

	LL_FLASH_EnableSleepPowerDown();

  HAL_PWREx_SetREGVDDHPAInputSupply(PWR_RADIO_REG_VDDHPA_VD11);
}
