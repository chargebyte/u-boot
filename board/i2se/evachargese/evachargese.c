// SPDX-License-Identifier: GPL-2.0+
/*
 * I2SE EVAcharge SE board
 *
 * Copyright (C) 2018 Michael Heimpold <michael.heimpold@i2se.com>
 * Copyright (C) 2020 in-tech smart charging GmbH
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/setup.h>
#include <fdt_support.h>
#include <linux/mii.h>
#include <miiphy.h>
#include <netdev.h>
#include <errno.h>
#include <fuse.h>

DECLARE_GLOBAL_DATA_PTR;

static u32 system_rev;
static u32 serialno;

/*
 * Functions
 */
int board_early_init_f(void)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;

	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);
	/* IO1 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK1, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);

	/* Disable Watchdog POR in order to avoid possible hangs after reset */
	setbits_le32(&clkctrl_regs->hw_clkctrl_reset, CLKCTRL_RESET_WDOG_POR_DISABLE);

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

#ifdef	CONFIG_CMD_MMC
int board_mmc_init(bd_t *bis)
{
	return mxsmmc_initialize(bis, 0, NULL, NULL);
}
#endif

#ifdef	CONFIG_CMD_NET
int board_eth_init(bd_t *bis)
{
	unsigned int reset_gpio = MX28_PAD_ENET0_RX_CLK__GPIO_4_13;
	int ret;

	ret = cpu_eth_init(bis);

	/* Reset PHY */
	gpio_request(reset_gpio, "enet0_phy_rst");
	gpio_direction_output(reset_gpio, 0);
	udelay(200);
	gpio_set_value(reset_gpio, 1);

	/* give PHY some time to get out of the reset */
	udelay(10000);

	ret = fecmxc_initialize_multi(bis, 0, 0, MXS_ENET0_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC\n");
		return ret;
	}

	return ret;
}

void mx28_adjust_mac(int dev_id, unsigned char *mac)
{
	mac[0] = 0x00;
	mac[1] = 0x01;
	mac[2] = 0x87;
}
#endif

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	uint8_t enetaddr[6];
	u32 mac = 0;

#ifdef CONFIG_MXS_OCOTP
	/* read MAC address for QCA7000 host interface */
	fuse_read(0, 1, &mac);
#endif

	if (mac != 0) {
		enetaddr[0] = 0x00;
		enetaddr[1] = 0x01;
		enetaddr[2] = 0x87;
		enetaddr[3] = (mac >> 16) & 0xff;
		enetaddr[4] = (mac >>  8) & 0xff;
		enetaddr[5] =  mac        & 0xff;

		fdt_find_and_setprop(blob,
		                     "/apb@80000000/apbh@80000000/ssp@80014000/qca7000@0",
		                     "local-mac-address", enetaddr, 6, 1);
	}

	return 0;
}
#endif

#ifdef CONFIG_REVISION_TAG
u32 get_board_rev(void)
{
	return system_rev;
}
#endif

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	serialnr->low = serialno;
	serialnr->high = 0;
}
#endif

int misc_init_r(void)
{
	unsigned int led_red_gpio = MX28_PAD_AUART1_TX__GPIO_3_5;
	char *s;

	/* enable red LED to indicate a running bootloader */
	gpio_request(led_red_gpio, "led_red");
	gpio_direction_output(led_red_gpio, 1);

	puts("Board: I2SE EVAcharge SE\n");

	serialno = env_get_ulong("serial#", 10, 0);
	s = env_get("serial#");
	if (s && s[0]) {
		puts("Serial: ");
		puts(s);
		putc('\n');
	}

	return 0;
}
