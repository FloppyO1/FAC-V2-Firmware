/*
 * fac_boot.c
 *
 *  Application side of the F.A.C. V2 USB CDC bootloader (FAC Tool spec firmware-bootloader.md):
 *  header read by the bootloader, vectors copied to SRAM, boot confirmation, USB command 8.
 */

#include "FAC_Code/fac_boot.h"
#include "FAC_Code/config.h"
#include "FAC_Code/fac_settings.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define FAC_BOOT_VECTOR_TABLE_SIZE	0xC0U	// 48 vectors of the STM32F072
#define FAC_BOOT_CONFIRM_DELAY_MS	1000U	// main loop time before the boot counts as good
#define FAC_BOOT_USB_QUERY			0
#define FAC_BOOT_USB_ENTER			1

/* Header read by the bootloader and the FAC Tool, placed at APP_BASE + 0xC0 by the linker script */
__attribute__((section(".fac_app_header"), used))
const FAC_boot_app_header_t fac_app_header = {
	.magic = FAC_BOOT_MAGIC_APP,
	.header_version = 1,
	.hw_id = FAC_BOOT_HW_ID,
	.fw_version = FIRMWARE_VERSION_VALUE,
	.link_address = FAC_BOOT_APP_BASE,
#ifdef DEBUG
	.flags = FAC_BOOT_APP_FLAG_DEBUG,	// the bootloader starts it without record, for ST-LINK debugging
#else
	.flags = 0,
#endif
};

extern uint32_t g_pfnVectors[];	// vector table of the startup file

static volatile FAC_boot_shared_t *const shared = (volatile FAC_boot_shared_t *) FAC_BOOT_SHARED_ADDR;

/* Check word of the shared block */
static uint32_t FAC_boot_shared_check(void) {
	const volatile uint32_t *w = (const volatile uint32_t *) shared;
	return ~(w[0] ^ w[1] ^ w[2] ^ w[3]);
}

static uint8_t FAC_boot_shared_is_valid(void) {
	return shared->magic == FAC_BOOT_MAGIC_SHARED && shared->check == FAC_boot_shared_check();
}

/**
 * @brief	Copy the vectors to SRAM and map SRAM at address 0 (Cortex-M0 has no VTOR)
 * @note	Call it first thing in main(), before HAL_Init() enables any interrupt
 */
void FAC_boot_relocate_vectors(void) {
	__disable_irq();
	memcpy((void *) SRAM_BASE, g_pfnVectors, FAC_BOOT_VECTOR_TABLE_SIZE);
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_SYSCFG_REMAPMEMORY_SRAM();
	__enable_irq();
}

/**
 * @brief	Tell the bootloader this boot went fine, once the main loop ran for a second
 * @note	Call it at every main loop cycle, it costs nothing once done
 */
void FAC_boot_confirm(void) {
	static uint8_t confirmed = FALSE;
	static uint32_t firstLoopTick = 0;
	if (confirmed)
		return;
	if (firstLoopTick == 0) {
		firstLoopTick = HAL_GetTick() | 1;	// never 0, which means "not started"
		return;
	}
	if (HAL_GetTick() - firstLoopTick < FAC_BOOT_CONFIRM_DELAY_MS)
		return;
	if (FAC_boot_shared_is_valid()) {
		shared->boot_attempts = 0;
		shared->check = FAC_boot_shared_check();
	}
	confirmed = TRUE;
}

/**
 * @brief	USB command 8 BOOTLOADER: [8, 0, 0, 0] query, [8, 1, 'B', 'L'] enter the bootloader
 * @retval	TRUE if the command was understood
 */
uint8_t FAC_boot_USB_command(uint8_t subCommand, const uint8_t *value) {
	static uint8_t reply[4];
	if (subCommand == FAC_BOOT_USB_QUERY) {
		const FAC_boot_bl_info_t *info = (const FAC_boot_bl_info_t *) FAC_BOOT_BL_INFO_ADDR;
		reply[0] = FAC_USB_COMMAND_BOOTLOADER;
		reply[1] = reply[2] = reply[3] = 0;	// no bootloader
		if (info->magic == FAC_BOOT_MAGIC_BL_INFO && info->hw_id == FAC_BOOT_HW_ID) {
			reply[1] = info->protocol_version;
			reply[2] = (uint8_t) (info->bl_version >> 8);	// big endian, as the rest of the protocol
			reply[3] = (uint8_t) info->bl_version;
		}
		CDC_Transmit_FS(reply, sizeof reply);
		return TRUE;
	}
	if (subCommand == FAC_BOOT_USB_ENTER && value[0] == 'B' && value[1] == 'L') {
		reply[0] = 73;	// ack
		CDC_Transmit_FS(reply, 1);
		HAL_Delay(50);	// let the ack leave before the USB goes away
		if (!FAC_boot_shared_is_valid()) {	// first use since power on: start clean
			memset((void *) shared, 0, sizeof(FAC_boot_shared_t));
			shared->magic = FAC_BOOT_MAGIC_SHARED;
		}
		shared->request = FAC_BOOT_REQUEST_STAY;
		shared->check = FAC_boot_shared_check();
		NVIC_SystemReset();	// the bootloader drives motors and servos low
	}
	return FALSE;
}
