/**
 * @file bms_usb.c
 * @brief USB clock recovery setup for the CDC-ACM shell port.
 *
 * The USB peripheral is clocked from the internal HSI48 oscillator, which is
 * not accurate enough for full-speed USB on its own. The Clock Recovery
 * System (CRS) trims HSI48 against the host's 1 kHz Start-Of-Frame packets.
 * Zephyr's STM32G4 clock driver does not configure the CRS, so it is done
 * here before the USB device stack starts.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_crs.h>

static int bms_usb_crs_init(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CRS);

	/* Reset-default reload / error limit already match 48 MHz vs 1 kHz SOF */
	LL_CRS_SetSyncSignalSource(LL_CRS_SYNC_SOURCE_USB);
	LL_CRS_EnableAutoTrimming();
	LL_CRS_EnableFreqErrorCounter();

	return 0;
}

SYS_INIT(bms_usb_crs_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
