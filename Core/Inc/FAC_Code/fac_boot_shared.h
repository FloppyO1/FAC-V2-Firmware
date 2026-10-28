/**
 * @file	fac_boot_shared.h
 * @brief	Memory map and structures shared by the F.A.C. V2 bootloader and the application.
 * @note	Copy this file unchanged into the application firmware. Specification: bootloader.md
 * 			of the FAC Tool (sections 3-5). Addresses and layouts must never change.
 */
#ifndef FAC_BOOT_SHARED_H
#define FAC_BOOT_SHARED_H

#include <stdint.h>

/* Addresses (bootloader.md, section 3) */
#define FAC_BOOT_BL_BASE          0x08000000UL
#define FAC_BOOT_BL_SIZE_MAX      0x00004000UL	// 16 KB reserved for the bootloader
#define FAC_BOOT_BL_INFO_ADDR     0x080000C0UL
#define FAC_BOOT_APP_BASE         0x08004000UL
#define FAC_BOOT_APP_MAX_SIZE     0x0001B800UL	// 110 KB, pages 8-62
#define FAC_BOOT_RECORD_ADDR      0x0801F800UL	// page 63
#define FAC_BOOT_PAGE_SIZE        2048UL
#define FAC_BOOT_SHARED_ADDR      0x200000C0UL
#define FAC_BOOT_VECTORS_SIZE     0xC0UL		// 48 vectors of the STM32F072
#define FAC_BOOT_APP_HEADER_ADDR  (FAC_BOOT_APP_BASE + FAC_BOOT_VECTORS_SIZE)

/* Magic values: 4 ASCII characters in memory order */
#define FAC_BOOT_MAGIC_BL_INFO    0x42434146UL	// "FACB"
#define FAC_BOOT_MAGIC_APP        0x41434146UL	// "FACA"
#define FAC_BOOT_MAGIC_RECORD     0x52434146UL	// "FACR"
#define FAC_BOOT_MAGIC_SHARED     0x53434146UL	// "FACS"
#define FAC_BOOT_REQUEST_STAY     0x59415453UL	// "STAY"

#define FAC_BOOT_HW_ID            2				// F.A.C. V2
#define FAC_BOOT_APP_FLAG_DEBUG   0x00000001UL	// header flag of Debug builds

/* Why the bootloader stayed active (shared block and INFO) */
enum {
	FAC_BOOT_STAY_NONE = 0,
	FAC_BOOT_STAY_NO_APP = 1,		// record missing or invalid
	FAC_BOOT_STAY_CRC_MISMATCH = 2,	// record valid, application CRC different
	FAC_BOOT_STAY_BAD_HEADER = 3,	// header missing or for another board/address
	FAC_BOOT_STAY_BAD_VECTORS = 4,	// stack or reset handler out of place
	FAC_BOOT_STAY_BOOT_FAILED = 5,	// 3 boots without confirmation
	FAC_BOOT_STAY_REQUESTED = 6,	// requested by the application
};

typedef struct {			// at APP_BASE + 0xC0, 16 bytes
	uint32_t magic;
	uint8_t header_version;	// 1
	uint8_t hw_id;
	uint16_t fw_version;	// FIRMWARE_VERSION_VALUE
	uint32_t link_address;	// FAC_BOOT_APP_BASE
	uint32_t flags;
} FAC_boot_app_header_t;

typedef struct {			// at 0x080000C0, 20 bytes, written by the bootloader build
	uint32_t magic;
	uint8_t info_version;
	uint8_t hw_id;
	uint16_t bl_version;
	uint32_t bl_size_max;
	uint32_t app_base;
	uint8_t protocol_version;
	uint8_t reserved[3];
} FAC_boot_bl_info_t;

typedef struct {			// at RECORD_ADDR, 20 bytes, written after a successful VERIFY
	uint32_t magic;
	uint32_t app_size;		// bytes from APP_BASE, multiple of 4
	uint32_t app_crc;		// CRC-32 (zlib) of the application
	uint16_t fw_version;	// copy of the header fw_version
	uint16_t flags;
	uint32_t record_crc;	// CRC-32 of the 16 bytes above
} FAC_boot_record_t;

typedef struct {			// at 0x200000C0, 32 bytes, survives a system reset
	uint32_t magic;
	uint32_t request;
	uint8_t boot_attempts;
	uint8_t stay_reason;
	uint16_t reserved;
	uint32_t reset_flags;	// RCC->CSR as the bootloader found it
	uint32_t check;			// ~(word0 ^ word1 ^ word2 ^ word3)
	uint32_t reserved2[3];
} FAC_boot_shared_t;

_Static_assert(sizeof(FAC_boot_app_header_t) == 16, "app header must be 16 bytes");
_Static_assert(sizeof(FAC_boot_bl_info_t) == 20, "bootloader info must be 20 bytes");
_Static_assert(sizeof(FAC_boot_record_t) == 20, "record must be 20 bytes");
_Static_assert(sizeof(FAC_boot_shared_t) == 32, "shared block must be 32 bytes");

#endif /* FAC_BOOT_SHARED_H */
