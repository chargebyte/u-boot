// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017-2018 I2SE GmbH
 * Copyright (C) 2020 in-tech smart charging GmbH
 */

#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/io.h>
#include <common.h>
#include <dm.h>
#include <env.h>
#include <fdt_support.h>
#include <fsl_esdhc_imx.h>
#include <fuse.h>
#include <miiphy.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-ci.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |              \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH  |                                   \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define MDIO_PAD_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |      \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST | PAD_CTL_ODE)

#define ENET_CLK_PAD_CTRL (PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS |                             \
	PAD_CTL_SPEED_MED   |                                   \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_LCD_CLK__UART4_DTE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_LCD_ENABLE__UART4_DTE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));

	return 0;
}

static iomux_v3_cfg_t const boardvariant_pads[] = {
	MX6_PAD_NAND_CLE__GPIO4_IO15   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NAND_CE0_B__GPIO4_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NAND_CE1_B__GPIO4_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NAND_DQS__GPIO4_IO16   | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#define BOARD_VARIANT_GPIO0	IMX_GPIO_NR(4, 15)
#define BOARD_VARIANT_GPIO1	IMX_GPIO_NR(4, 13)
#define BOARD_VARIANT_GPIO2	IMX_GPIO_NR(4, 14)
#define BOARD_VARIANT_GPIO3	IMX_GPIO_NR(4, 16)

static iomux_v3_cfg_t const usdhc2_emmc_rst_pad[] = {
	/* RST_B */
	MX6_PAD_NAND_ALE__GPIO4_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
};

#define USDHC2_RST_GPIO	IMX_GPIO_NR(4, 10)

int board_revision_is_v0r3;

static enum board_variants {
	TARRAGON_DEFAULT,
	TARRAGON_MASTER,
	TARRAGON_SLAVE,
	TARRAGON_MICRO,
	TARRAGON_SLAVEXT,
	/* insert new variants before this line */
	TARRAGON_UNKNOWN,
} board_variant;

static const char * const board_variants_names[] = {
	"Default",
	"Master",
	"Slave",
	"Micro",
	"SlaveXT",
	/* insert new variant names before this line */
	"Unknown",
};

static const char * const board_variants_default_dtb_names[] = {
	NULL,
	"imx6ull-tarragon-master.dtb",
	"imx6ull-tarragon-slave.dtb",
	"imx6ull-tarragon-micro.dtb",
	"imx6ull-tarragon-slavext.dtb",
	/* insert new variant names before this line */
	NULL
};

static void boardvariants_init(void)
{
	/* detect board revision first */
	imx_iomux_v3_setup_multiple_pads(usdhc2_emmc_rst_pad, ARRAY_SIZE(usdhc2_emmc_rst_pad));

	gpio_request(USDHC2_RST_GPIO, "board_revision");
	gpio_direction_input(USDHC2_RST_GPIO);
	board_revision_is_v0r3 = !gpio_get_value(USDHC2_RST_GPIO);
	gpio_free(USDHC2_RST_GPIO);

	/* detect board variant */
	imx_iomux_v3_setup_multiple_pads(boardvariant_pads, ARRAY_SIZE(boardvariant_pads));

	gpio_request(BOARD_VARIANT_GPIO0, "board_variant0");
	gpio_request(BOARD_VARIANT_GPIO1, "board_variant1");
	gpio_request(BOARD_VARIANT_GPIO2, "board_variant2");
	gpio_request(BOARD_VARIANT_GPIO3, "board_variant3");

	gpio_direction_input(BOARD_VARIANT_GPIO0);
	gpio_direction_input(BOARD_VARIANT_GPIO1);
	gpio_direction_input(BOARD_VARIANT_GPIO2);
	gpio_direction_input(BOARD_VARIANT_GPIO3);

	/* reverse GPIO order, because schematics have a special assignment */
	board_variant =
		gpio_get_value(BOARD_VARIANT_GPIO3) << 0 |
		gpio_get_value(BOARD_VARIANT_GPIO2) << 1 |
		gpio_get_value(BOARD_VARIANT_GPIO1) << 2 |
		gpio_get_value(BOARD_VARIANT_GPIO0) << 3;

	/* in case hardware is populated wrong or variant is not (yet) known */
	if (board_variant > TARRAGON_UNKNOWN)
		board_variant = TARRAGON_UNKNOWN;
}

#ifdef CONFIG_FEC_MXC

static iomux_v3_cfg_t const fec1_pads[] = {
	/* ENET1 MDIO bus connects PHY for ENET1 */
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),

	/* ENET1 RMII */
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),

	/* ENET1 PHY reset */
	MX6_PAD_SNVS_TAMPER6__GPIO5_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
};

#define ENET1_PHY_RST_GPIO	IMX_GPIO_NR(5, 6)

int board_eth_init(bd_t *bis)
{
	imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));

	gpio_request(ENET1_PHY_RST_GPIO, "enet1_phy_rst");

	/* reset PHY */
	gpio_direction_output(ENET1_PHY_RST_GPIO, 0);
	udelay(200);
	gpio_set_value(ENET1_PHY_RST_GPIO, 1);

	/* give PHY some time to get out of the reset */
	udelay(10000);

	return fecmxc_initialize_multi(bis, 0,
	                               CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	uint8_t enetaddr[6];
	u32 mac = 0;

	enetaddr[0] = 0x00;
	enetaddr[1] = 0x01;
	enetaddr[2] = 0x87;

	/* read QCA7000 MAINS MAC from OCOTP_GP1 */
	fuse_read(4, 6, &mac);

	if (mac != 0) {
		enetaddr[3] = (mac >> 16) & 0xff;
		enetaddr[4] = (mac >>  8) & 0xff;
		enetaddr[5] =  mac        & 0xff;

		fdt_find_and_setprop(blob,
		                     "spi3/ethernet@0",
		                     "local-mac-address", enetaddr, 6, 1);
	}

	/* reset mac to detect errors when fuse_read fails */
	mac = 0;

	/* read QCA7000 CP MAC from OCOTP_GP2 */
	fuse_read(4, 7, &mac);

	if (mac != 0) {
		enetaddr[3] = (mac >> 16) & 0xff;
		enetaddr[4] = (mac >>  8) & 0xff;
		enetaddr[5] =  mac        & 0xff;

		fdt_find_and_setprop(blob,
		                     "spi1/ethernet@0",
		                     "local-mac-address", enetaddr, 6, 1);
	}

	if (!board_revision_is_v0r3) {
		/* for board revision > V0R3: switch off wdog1 and enable wdog2 */
		fdt_set_status_by_alias(blob, "/soc/aips-bus@02000000/wdog@020bc000", FDT_STATUS_DISABLED, 0);
		fdt_set_status_by_alias(blob, "/soc/aips-bus@02000000/wdog@020c0000", FDT_STATUS_OKAY, 0);

		/* enable USB on extension header */
		fdt_set_status_by_alias(blob, "/soc/aips-bus@02100000/usb@02184200", FDT_STATUS_OKAY, 0);
	}

	return 0;
}
#endif

static int setup_fec(int fec_id)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	/*
	 * Use 50M anatop loopback REF_CLK1 for ENET1,
	 * clear gpr1[13], set gpr1[17].
	 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
	                IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);

	ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	return 0;
}
#endif

static iomux_v3_cfg_t const qca7000_cp_pads[] = {
	MX6_PAD_UART4_RX_DATA__ECSPI2_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_UART4_TX_DATA__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_UART5_RX_DATA__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_UART5_TX_DATA__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),

	/* RST pin */
	MX6_PAD_LCD_DATA12__GPIO3_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
};

#define QCA7000_CP_RST_GPIO	IMX_GPIO_NR(3, 17)

static iomux_v3_cfg_t const qca7000_mains_pads[] = {
	MX6_PAD_ENET2_RX_ER__ECSPI4_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ENET2_TX_EN__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),

	/* RST pin */
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
};

#define QCA7000_MAINS_RST_GPIO	IMX_GPIO_NR(5, 7)

static void qca7000_init(void)
{
	if (board_variant == TARRAGON_MASTER ||
	    board_variant == TARRAGON_SLAVE ||
	    board_variant == TARRAGON_SLAVEXT) {
		/* setup pin muxing */
		imx_iomux_v3_setup_multiple_pads(qca7000_cp_pads, ARRAY_SIZE(qca7000_cp_pads));
		/* request reset GPIO */
		gpio_request(QCA7000_CP_RST_GPIO, "qca7000_cp_rst");
		/* de-assert reset */
		gpio_direction_output(QCA7000_CP_RST_GPIO, 1);
	}

	if (board_variant == TARRAGON_MASTER) {
		/* setup pin muxing */
		imx_iomux_v3_setup_multiple_pads(qca7000_mains_pads, ARRAY_SIZE(qca7000_mains_pads));
		/* request reset GPIO */
		gpio_request(QCA7000_MAINS_RST_GPIO, "qca7000_mains_rst");
		/* de-assert reset */
		gpio_direction_output(QCA7000_MAINS_RST_GPIO, 1);
	}
}

static iomux_v3_cfg_t const motor_1_drv_pads[] = {
	MX6_PAD_LCD_DATA02__GPIO3_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
	MX6_PAD_LCD_DATA03__GPIO3_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
};

#define MOTOR_1_DRV_IN1	IMX_GPIO_NR(3, 7)
#define MOTOR_1_DRV_IN2	IMX_GPIO_NR(3, 8)

static void motor_1_drv_init(void)
{
	imx_iomux_v3_setup_multiple_pads(motor_1_drv_pads, ARRAY_SIZE(motor_1_drv_pads));

	gpio_request(MOTOR_1_DRV_IN1, "motor1_drv_in1");
	gpio_request(MOTOR_1_DRV_IN2, "motor1_drv_in2");

	/* start charging caps */
	gpio_direction_output(MOTOR_1_DRV_IN1, 1);
	gpio_direction_output(MOTOR_1_DRV_IN2, 0);
}

static iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_LCD_DATA09__GPIO3_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
	MX6_PAD_LCD_DATA10__GPIO3_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
	MX6_PAD_LCD_DATA14__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL) | MUX_MODE_SION,
};

#define LED_GREEN	IMX_GPIO_NR(3, 14)
#define LED_YELLOW	IMX_GPIO_NR(3, 15)
#define LED_RED		IMX_GPIO_NR(3, 19)

static void led_init(void)
{
	imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));

	/* request GPIO for LEDs */
	gpio_request(LED_GREEN, "led_green");
	gpio_request(LED_YELLOW, "led_yellow");
	gpio_request(LED_RED, "led_red");

	/* enable red LED to indicate a running bootloader */
	gpio_direction_output(LED_GREEN, 0);
	gpio_direction_output(LED_YELLOW, 0);
	gpio_direction_output(LED_RED, 1);
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	/* run board variant detection first to populate board_variant variable */
	boardvariants_init();
	printf("Board: in-tech Tarragon %s (%s)\n",
	       board_variants_names[board_variant],
	       board_revision_is_v0r3 ? "V0R3" : "V0R4/V0R5");

	motor_1_drv_init();

	qca7000_init();

	led_init();

#ifdef	CONFIG_FEC_MXC
	if (board_variant == TARRAGON_MASTER ||
	    board_variant == TARRAGON_SLAVE ||
	    board_variant == TARRAGON_SLAVEXT)
		setup_fec(CONFIG_FEC_ENET_DEV);
#endif

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

int show_board_info(void)
{
	switch (get_boot_device()) {
	case SD2_BOOT:
		puts("Boot:  SD2\n");
		break;
	case MMC2_BOOT:
		puts("Boot:  MMC2\n");
		break;
	case USB_BOOT:
		puts("Boot:  USB\n");
		break;
	default:
		/* no output */
		;
	}

	return 0;
}

int misc_init_r(void)
{
	char *s;

	env_set("board_variant", board_variants_names[board_variant]);
	env_set("board_revision", board_revision_is_v0r3 ? "V0R3" : "V0R4/V0R5");
	env_set("reset_cause", get_reset_cause());

	/* guess DT blob if not already set in environment */
	if (!env_get("fdt_file") && board_variants_default_dtb_names[board_variant] != NULL)
		env_set("fdt_file", board_variants_default_dtb_names[board_variant]);

	/* print serial number if available */
	s = env_get("serial#");
	if (s && s[0]) {
		puts("Serial: ");
		puts(s);
		puts("\n");
	}

	return 0;
}
