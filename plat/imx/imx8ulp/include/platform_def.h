/*
 * Copyright 2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLATFORM_DEF_H
#define PLATFORM_DEF_H

#include <lib/utils_def.h>

#define PLATFORM_LINKER_FORMAT		"elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH		aarch64

#define PLATFORM_STACK_SIZE		0x400
#define CACHE_WRITEBACK_GRANULE		64

#define PLAT_PRIMARY_CPU		0x0
#define PLATFORM_MAX_CPU_PER_CLUSTER	2
#define PLATFORM_CLUSTER_COUNT		1
#define PLATFORM_CORE_COUNT		2
#define PLATFORM_CLUSTER0_CORE_COUNT	2
#define PLATFORM_CLUSTER1_CORE_COUNT	0

#define IMX_PWR_LVL0			MPIDR_AFFLVL0
#define IMX_PWR_LVL1			MPIDR_AFFLVL1
#define IMX_PWR_LVL2			MPIDR_AFFLVL2

#define PWR_DOMAIN_AT_MAX_LVL		U(1)
#define PLAT_MAX_PWR_LVL		U(2)

#define PLAT_SLEEP_RET_STATE		U(1)
#define PLAT_DEEP_SLEEP_RET_STATE	U(2)
#define PLAT_MAX_RET_STATE		U(3)

#define PLAT_POWER_DOWN_OFF_STATE	U(4)
#define PLAT_DEEP_POWER_DOWN_STATE	U(5)
#define PLAT_MAX_OFF_STATE		U(6)

#define BL31_BASE			0x20040000
#define BL31_LIMIT			0x20070000

#define PLAT_VIRT_ADDR_SPACE_SIZE	(1ull << 32)
#define PLAT_PHY_ADDR_SPACE_SIZE	(1ull << 32)

#define MAX_XLAT_TABLES			8
#define MAX_MMAP_REGIONS		9

#define PLAT_GICD_BASE			U(0x2d400000)
#define PLAT_GICR_BASE			U(0x2d440000)
#define DEVICE0_BASE			U(0x20000000)
#define DEVICE0_SIZE			U(0x10000000)
#define DEVICE1_BASE			U(0x30000000)
#define DEVICE1_SIZE			U(0x10000000)
#define IMX_LPUART4_BASE		U(0x29390000)
#define IMX_LPUART5_BASE		U(0x293a0000)
#define IMX_LPUART_BASE			IMX_LPUART5_BASE
#define IMX_CAAM_BASE			U(0x292e0000)
#define IMX_BOOT_UART_CLK_IN_HZ		24000000
#define IMX_CONSOLE_BAUDRATE		115200

#define IMX_CGC1_BASE			U(0x292c0000)
#define IMX_PCC3_BASE			U(0x292d0000)
#define IMX_PCC4_BASE			U(0x29800000)
#define IMX_CGC2_BASE			U(0x2da60000)
#define IMX_PCC5_BASE			U(0x2da70000)
#define IMX_CMC1_BASE			U(0x29240000)
#define IMX_WUU1_BASE			U(0x29260000)
#define IMX_SIM1_BASE			U(0x29290000)
#define IMX_GPIOD_BASE			U(0x2e200000)
#define IMX_GPIOE_BASE			U(0x2d000000)
#define IMX_GPIOF_BASE			U(0x2d010000)
#define IMX_WDOG3_BASE			U(0x292a0000)
#define IOMUXC_PTD_PCR_BASE		U(0x298c0000)
#define IOMUXC_PTE_PCR_BASE		U(0x298c0080)
#define IOMUXC_PTF_PCR_BASE		U(0x298c0100)
#define IOMUXC_PSMI_BASE0		U(0x298c0800)
#define IOMUXC_PSMI_BASE1		U(0x298c0838)
#define IOMUXC_PSMI_BASE2		U(0x298c0954)
#define IOMUXC_PSMI_BASE3		U(0x298c0994)
#define IOMUXC_PSMI_BASE4		U(0x298c0a58)

#define IMX_ROM_ENTRY			U(0x1000)
#define COUNTER_FREQUENCY		1000000

#define PLAT_NS_IMAGE_OFFSET		0x80200000

#define BL32_FDT_OVERLAY_ADDR		0x9d000000

#define IMX_TRUSTY_STACK_SIZE 0x100

#endif /* PLATFORM_DEF_H */
