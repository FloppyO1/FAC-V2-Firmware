/*
 * fac_boot.h
 *
 *  Application side of the F.A.C. V2 USB CDC bootloader: vector relocation,
 *  boot confirmation and USB command 8 (FAC Tool spec firmware-bootloader.md).
 */

#ifndef INC_FAC_CODE_FAC_BOOT_H_
#define INC_FAC_CODE_FAC_BOOT_H_

#include <stdint.h>
#include "FAC_Code/fac_boot_shared.h"	// copied unchanged from the bootloader repository

void FAC_boot_relocate_vectors(void);	// first thing in main(), before HAL_Init()
void FAC_boot_confirm(void);			// every main loop cycle
uint8_t FAC_boot_USB_command(uint8_t subCommand, const uint8_t *value);	// USB command 8

#endif /* INC_FAC_CODE_FAC_BOOT_H_ */
