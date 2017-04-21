/*
 * Copyright (C) 2014-2015 Michael Heimpold <mhei@heimpold.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIGS_DUCKBILL_H__
#define __CONFIGS_DUCKBILL_H__

/* System configurations */
#define CONFIG_MX28				/* i.MX28 SoC */
#define CONFIG_MACH_TYPE	MACH_TYPE_DUCKBILL

#define CONFIG_MISC_INIT_R

#define CONFIG_SYS_MXS_VDD5V_ONLY

/* U-Boot Commands */
#define CONFIG_CMD_FUSE
#define CONFIG_CMD_I2C
#define CONFIG_CMD_UNZIP

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Environment is in MMC */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_IS_IN_MMC		1
#define CONFIG_ENV_SIZE			(128 * 1024)
#define CONFIG_ENV_OFFSET		(128 * 1024)
#define CONFIG_ENV_OFFSET_REDUND	(256 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT

/* FEC Ethernet on SoC */
#ifdef	CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_NET_MULTI
#define CONFIG_MX28_FEC_MAC_IN_OCOTP
#define CONFIG_FEC_MXC_PHYADDR	1
#define IMX_FEC_BASE		MXS_ENET0_BASE
#endif

#define CONFIG_IPADDR		192.168.1.10
#define CONFIG_SERVERIP		192.168.1.1
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_GATEWAYIP	192.168.1.254

/* BOOTP options */
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME

/* SPI */
#ifdef CONFIG_CMD_SPI
#define CONFIG_DEFAULT_SPI_BUS		2
#define CONFIG_DEFAULT_SPI_MODE		SPI_MODE_0
#endif

/* Boot Linux */
#define CONFIG_BOOTDELAY	1
#define CONFIG_BOOTFILE		"zImage"
#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR
#define CONFIG_REVISION_TAG
#define CONFIG_SERIAL_TAG
#define CONFIG_OF_BOARD_SETUP
#define CONFIG_BOOT_RETRY_TIME		120	/* retry autoboot after 120 seconds */
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT		"Autobooting in %d seconds, " \
					"press <c> to stop\n"
#define CONFIG_AUTOBOOT_DELAY_STR	"\x63"	/* allows retry after retry time */
#define CONFIG_AUTOBOOT_STOP_STR	" "	/* stop autoboot with <Space> */
#define CONFIG_RESET_TO_RETRY			/* reset board to retry booting */

/* Extra Environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"mmc_part2_offset=1000\0" \
	"mmc_part3_offset=19000\0" \
	"update_openwrt_firmware_filename=openwrt-mxs-root.ext4\0" \
	"update_openwrt_firmware=" \
		"if mmc rescan; then " \
			"if tftp ${update_openwrt_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} + 1ff; " \
				"setexpr fw_sz ${fw_sz} / 200; " \
				"mmc write ${loadaddr} ${mmc_part2_offset} ${fw_sz}; " \
				"mmc write ${loadaddr} ${mmc_part3_offset} ${fw_sz}; " \
			"fi; " \
		"fi\0" \
	"update_fw_filename_prefix=emmc.img.\0" \
	"update_fw_filename_suffix=.gz\0" \
	"update_fw_parts=0x6\0" \
	"update_fw_fsize_uncompressed=4000000\0" \
	"gzwrite_wbuf=100000\0" \
	"update_emmc_firmware=" \
		"setexpr i ${update_fw_parts}; setexpr error 0; " \
		"while itest ${i} -gt 0; do " \
			"echo Transfering firmware image part ${i} of ${update_fw_parts}; " \
			"if itest ${i} -le 9; then " \
				"setenv j 0${i}; " \
			"else " \
				"setenv j ${i}; " \
			"fi; " \
			"if tftp ${loadaddr} ${update_fw_basedir}${update_fw_filename_prefix}${j}${update_fw_filename_suffix}; then " \
				"setexpr k ${i} - 1; " \
				"setexpr offset ${update_fw_fsize_uncompressed} * ${k}; " \
				"if gzwrite mmc ${mmcdev} ${loadaddr} ${filesize} ${gzwrite_wbuf} ${offset}; then " \
					"setexpr i ${i} - 1; " \
				"else " \
					"setexpr i 0; " \
					"setexpr error 1; " \
				"fi; " \
			"else " \
				"setexpr i 0; " \
				"setexpr error 1; " \
			"fi; " \
		"done; setenv i; setenv j; setenv k; setenv fsize; setenv filesize; setenv offset; " \
		"if test ${error} -eq 1; then " \
			"echo Firmware Update FAILED; " \
		"else " \
			"echo Firmware Update OK; " \
		"fi; setenv error\0" \
	"erase_mmc=mmc erase 0 2\0" \
	"erase_env1=mmc erase 100 100\0" \
	"erase_env2=mmc erase 200 100\0" \
	"image=zImage\0" \
	"console=ttyAMA0\0" \
	"fdt_file=imx28-duckbill-2.dtb\0" \
	"fdt_addr=0x41000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"bootsys=1\0" \
	"mmcdev=0\0" \
	"mmcpart=2\0" \
	"mmcroot=/dev/mmcblk0p2\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot} " \
		"rootwait bootsys=${bootsys} panic=1\0" \
	"loadimage=ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/${image}\0" \
	"loadfdt=ext4load mmc ${mmcdev}:${mmcpart} ${fdt_addr} /boot/${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"setexpr mmcpart 1 + ${bootsys}; " \
		"setenv mmcroot /dev/mmcblk0p${mmcpart}; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi\0" \
	"nfsroot=/\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; "	\
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi;" \
			"fi; " \
		"else " \
			"bootz; " \
		"fi\0"

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev}; " \
	"if mmc rescan; then " \
		"if run loadimage; then " \
			"run mmcboot; " \
		"else " \
			"run netboot; " \
		"fi; " \
	"else " \
		"run netboot; " \
	"fi"

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_DUCKBILL_H__ */
