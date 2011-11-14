/*
 * linux/arch/arm/mach-omap2/board-omap3egf.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "board-flash.h"

/* CPLD GPIO */
#include <linux/spi/spi.h>
#include <linux/gpio_spi.h>

#define OMAP3_EGF_DISPLAY_ENABLE_GPIO			213
#define OMAP3_EGF_LCD_3V3_ENABLE_GPIO 			2

/* EEPROM */
#include <linux/i2c/at24.h>
#define EEPROM_ON_MODULE_I2C_ADDR 0x50
#define EEPROM_ON_BOARD_I2C_ADDR  0x56


/* TOUCHSCREEN */
#if defined(CONFIG_TOUCHSCREEN_SX8652)

static int sx8652_pendown_irq(void)
{
  return gpio_to_irq (OMAP3_EGF_TS_GPIO);
}

static int sx8652_pendown_state(void)
{
	return !gpio_get_value(OMAP3_EGF_TS_GPIO);
}

static struct sx8652_platform_data sx_info ={
		.model				= 8652,
		.x_min				= 150,
		.x_max				= 3830,
		.y_min				= 190,
		.y_max				= 3830,
		.x_plate_ohms		= 450,
		.y_plate_ohms		= 250,
		.pressure_max		= 15000,
		.get_pendown_state	= sx8652_pendown_state,
		.get_pendown_irq	= sx8652_pendown_irq,
};

static void __init egf_ts_init(void)
{
	int r;
	omap_mux_init_gpio(OMAP3_EGF_TS_GPIO, OMAP_PIN_INPUT);
	r = gpio_request_one(OMAP3_EGF_TS_GPIO, GPIOF_IN,"Pendown Ts Gpio");
	if (r < 0)
		printk(KERN_ERR "Unable to get Pendown IRQ GPIO\n");
}
#endif

/* Lcd Pwm Backlight defines */

#define TWL_INTBR_PMBR1	0xD
#define TWL_INTBR_GPBR1	0xC
#define TWL_LED_PWMON	0x0
#define TWL_LED_PWMOFF	0x1

#define OMAP3_EGF_LCD_3V3_ENABLE_GPIO 2
#define DISPLAY_EN_GPIO 213

static struct spi_board_info  egf_gpio_spi[] = {
		/* GPIO SPI EXPANDER CPLD */
  {
    .modalias	= "gpio_spi",
    .max_speed_hz	= 1000000, //1 MHz
    .bus_num	= 1,
    .chip_select	= 0,
    .mode = SPI_MODE_0,
  },
		/* TOUCHSCREEN */
#if defined(CONFIG_TOUCHSCREEN_SX8652)
  {
    .modalias	= "sx8652",
    .max_speed_hz	= 1000000, //1 MHz
    .bus_num	= 1,
    .chip_select	= 2,
    .max_speed_hz = 125000*16,
    .platform_data = &sx_info,
  },
#endif
};


/* DSS */

static int egf_enable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void egf_disable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct omap_dss_device egf_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio = OMAP3_EGF_DISPLAY_ENABLE_GPIO,
	.platform_enable = egf_enable_dvi,
	.platform_disable = egf_disable_dvi,
};

static struct omap_dss_device egf_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *egf_dss_devices[] = {
	&egf_dvi_device,
	&egf_tv_device,
};

static struct omap_dss_board_info egf_dss_data = {
	.num_devices = ARRAY_SIZE(egf_dss_devices),
	.devices = egf_dss_devices,
	.default_device = &egf_dvi_device,
};

static struct platform_device egf_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &egf_dss_data,
	},
};

static struct regulator_consumer_supply egf_vdac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss");

static struct regulator_consumer_supply egf_vdvi_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");

static void __init egf_display_init(void)
{
	int r;

	r = gpio_request(egf_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
		return;
	}

	gpio_direction_output(egf_dvi_device.reset_gpio, 0);

	r = gpio_request(egf_dvi_device.reset_gpio, "DVI reset");
		if (r < 0) {
			printk(KERN_ERR "Unable to get LCD 3V3 Enable GPIO\n");
			return;
		}

	gpio_direction_output(OMAP3_EGF_LCD_3V3_ENABLE_GPIO, 1);

}

/* EEPROM  */

/* EEprom on SOM336 */
static struct at24_platform_data at24c64 = {
     .byte_len       = SZ_64K / 8,
     .flags			 = AT24_FLAG_ADDR16,
     .page_size      = 32,
};


static struct i2c_board_info __initdata egf_i2c_eeprom_on_module[] = {
       {
               I2C_BOARD_INFO("24c64", EEPROM_ON_MODULE_I2C_ADDR),
               .platform_data  = &at24c64,
       },
};

/* EEprom on JSF0377 */
static struct i2c_board_info __initdata egf_i2c_eeprom_on_board[] = {
       {
               I2C_BOARD_INFO("24c04", EEPROM_ON_BOARD_I2C_ADDR),
       },
};

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA ,
		.gpio_wp	= -EINVAL,
	},
	{
    	.mmc		= 2,
    	.caps		= MMC_CAP_4_BIT_DATA,
    	.gpio_wp	= -EINVAL,
		.ext_clock	= 1,
		.transceiver	= true,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply egf_vmmc1_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.0");

static struct regulator_consumer_supply egf_vmmc2_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1");

static struct regulator_consumer_supply egf_vsim_supply = {
	.supply			= "vmmc_aux",
};


static struct gpio_led gpio_leds[];

static int egf_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	mmc[1].gpio_cd = gpio + 1;
	omap2_hsmmc_init(mmc);



	return 0;
}

static struct twl4030_gpio_platform_data egf_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= egf_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data egf_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &egf_vmmc1_supply,
};
/* VMMC2 for MMC2 pins CMD, CLK, DAT0..DAT3 (max 100 mA) */
static struct regulator_init_data egf_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &egf_vmmc2_supply,
};
/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data egf_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &egf_vsim_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data egf_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &egf_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data egf_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &egf_vdvi_supply,
};


static struct twl4030_usb_data egf_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data egf_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data egf_codec_data = {
	.audio_mclk = 26000000,
	.audio = &egf_audio_data,
};

static struct twl4030_platform_data egf_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &egf_usb_data,
	.gpio		= &egf_gpio_data,
	.codec		= &egf_codec_data,
	.vmmc1		= &egf_vmmc1,
	.vmmc2		= &egf_vmmc2,
	.vsim		= &egf_vsim,
	.vdac		= &egf_vdac,
	.vpll2		= &egf_vpll2,
};

static struct i2c_board_info __initdata egf_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &egf_twldata,
	},
};

static int __init omap3_egf_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, egf_i2c_boardinfo,
			ARRAY_SIZE(egf_i2c_boardinfo));

	/* Bus 2 is used for Camera/Sensor interface */
	omap_register_i2c_bus(2, 400, egf_i2c_eeprom_on_module, ARRAY_SIZE(egf_i2c_eeprom_on_module));
	omap_register_i2c_bus(3, 400, egf_i2c_eeprom_on_board, ARRAY_SIZE(egf_i2c_eeprom_on_board));


	return 0;
}

/* Funzione di inizializzazione SPI */
static void __init omap3_egf_spi_init(void)
{
  printk("Registering SPI boards\n");
  spi_register_board_info(egf_gpio_spi, ARRAY_SIZE(egf_gpio_spi));
}


static void __init omap3_egf_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);
	omap_init_irq();
	gpmc_init();
	omap2_gp_clockevent_set_gptimer(1);

}

static struct platform_device *omap3_egf_devices[] __initdata = {
	&egf_dss_device,
};


static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 16,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static void __init omap3_egf_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_egf_spi_init();
	omap3_egf_i2c_init();
	platform_add_devices(omap3_egf_devices,
			ARRAY_SIZE(omap3_egf_devices));
	omap_serial_init();


	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);

	egf_display_init();
}

MACHINE_START(OMAP3_EGF, "OMAP3 EGF")
	/* Maintainer: Andrea Collamati - http://www.elettronicagf.it */
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_egf_init_irq,
	.init_machine	= omap3_egf_init,
	.timer		= &omap_timer,
MACHINE_END
