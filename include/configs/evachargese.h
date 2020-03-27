/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 Michael Heimpold <michael.heimpold@i2se.com>
 * Copyright (C) 2020 in-tech smart charging GmbH
 */

#ifndef __CONFIGS_EVACHARGESE_H__
#define __CONFIGS_EVACHARGESE_H__

/* System configurations */
#define CONFIG_MISC_INIT_R

#define CONFIG_SYS_MXS_VDD5V_ONLY

/* Memory configuration */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Environment is in MMC */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_MMC_ENV_DEV		0

/* FEC Ethernet on SoC */
#ifdef CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_NET_MULTI
#define CONFIG_MX28_FEC_MAC_IN_OCOTP
#define IMX_FEC_BASE		MXS_ENET0_BASE
#endif

#define CONFIG_IPADDR		192.168.1.10
#define CONFIG_SERVERIP		192.168.1.1
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_GATEWAYIP	192.168.1.254

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
	"update_fw_filename_prefix=emmc.img.\0" \
	"update_fw_filename_suffix=.gz\0" \
	"update_fw_parts=0x6\0" \
	"update_fw_fsize_uncompressed=4000000\0" \
	"gzwrite_wbuf=100000\0" \
	"update_emmc_firmware=" \
		"setexpr i ${update_fw_parts}; setexpr error 0; " \
		"while itest ${i} -gt 0; do " \
			"echo Transferring firmware image part ${i} of ${update_fw_parts}; " \
			"if itest ${i} -le f; then " \
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
	"image=zImage\0" \
	"console=ttyAMA0\0" \
	"fdt_file=imx28-evachargese.dtb\0" \
	"fdt_addr=0x41000000\0" \
	"ip_dyn=yes\0" \
	"BOOT_ORDER=A B\0" \
	"BOOT_A_LEFT=3\0" \
	"BOOT_B_LEFT=3\0" \
	"mmcdev=0\0" \
	"mmcpart=2\0" \
	"mmcroot=/dev/mmcblk0p2\0" \
	"mmcargs=setenv bootargs ${bootargs} console=${console},${baudrate} panic=1\0" \
	"common_mmcargs=rootfstype=ext4 rootwait\0" \
	"loadimage=load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/${image}\0" \
	"loadfdt=load mmc ${mmcdev}:${mmcpart} ${fdt_addr} /boot/${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"test -n \"${BOOT_ORDER}\" || setenv BOOT_ORDER \"A B\"; " \
		"test -n \"${BOOT_A_LEFT}\" || setenv BOOT_A_LEFT 3; " \
		"test -n \"${BOOT_B_LEFT}\" || setenv BOOT_B_LEFT 3; " \
		"setenv bootargs; " \
		"for BOOT_SLOT in \"${BOOT_ORDER}\"; do " \
			"if test \"x${bootargs}\" != x; then " \
				"; " \
			"elif test x${BOOT_SLOT} = xA; then " \
				"if test ${BOOT_A_LEFT} -gt 0; then " \
					"setexpr BOOT_A_LEFT ${BOOT_A_LEFT} - 1; " \
					"echo \"Found valid slot A, ${BOOT_A_LEFT} attempts remaining\"; " \
					"setenv mmcpart 2; " \
					"setenv mmcroot /dev/mmcblk0p${mmcpart}; " \
					"setenv bootargs ${common_mmcargs} root=${mmcroot} rauc.slot=A; " \
				"fi; " \
			"elif test \"x${BOOT_SLOT}\" = xB; then " \
				"if test ${BOOT_B_LEFT} -gt 0; then " \
					"setexpr BOOT_B_LEFT ${BOOT_B_LEFT} - 1; " \
					"echo \"Found valid slot B, ${BOOT_B_LEFT} attempts remaining\"; " \
					"setenv mmcpart 3; " \
					"setenv mmcroot /dev/mmcblk0p${mmcpart}; " \
					"setenv bootargs ${common_mmcargs} root=${mmcroot} rauc.slot=B; " \
				"fi; " \
			"fi; " \
		"done; " \
		"if test -n \"${bootargs}\"; then " \
			"saveenv; " \
		"else " \
			"echo \"No valid slot found, resetting tries to 3\"; " \
			"setenv BOOT_A_LEFT 3; " \
			"setenv BOOT_B_LEFT 3; " \
			"saveenv; " \
			"reset; " \
		"fi; " \
		"run mmcargs; " \
		"run loadimage; run loadfdt; " \
		"bootz ${loadaddr} - ${fdt_addr}\0" \
	"nfsroot=/\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"${get_cmd} ${fdt_addr} ${fdt_file}; " \
		"bootz ${loadaddr} - ${fdt_addr}\0"

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev}; " \
	"if mmc rescan; then " \
		"run mmcboot; " \
	"else " \
		"run netboot; " \
	"fi"

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_EVACHARGESE_H__ */
