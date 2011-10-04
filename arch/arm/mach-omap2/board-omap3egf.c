/*
 * linux/arch/arm/mach-omap2/board-omap3egf.c
 *
 * Copyright (C) 2011 Elettronica GF s.r.l.
 * Author: Andrea Collamati <andrea.collamati@elettronicagf.it>
 * Author: Stefano Donati <stefano.donati@elettronicagf.it>
 *
 * Modified from mach-omap/omap1/board-generic.c
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
#include <linux/opp.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>
#include <linux/spi/spi.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <linux/gpio_spi.h>
#include "mux.h"
#include "hsmmc.h"
#include "pm.h"
#include "common-board-devices.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#ifdef CONFIG_TOUCHSCREEN_SX8652
#define	OMAP3_EGF_TS_GPIO 114
#include <linux/spi/sx8652.h>
#endif
#include <linux/i2c/at24.h>
#define EEPROM_I2C_ADDR 0x50

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

#define LCD_3V3_ENABLE_GPIO 2
#define DISPLAY_EN_GPIO 213

/****************** GPIO SPI EXPANDER CPLD **********************/
static struct spi_board_info  egf_gpio_spi[] = {
  {
    .modalias	= "gpio_spi",
    .max_speed_hz	= 1000000, //1 MHz
    .bus_num	= 1,
    .chip_select	= 0,
    .mode = SPI_MODE_0,
  },
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

static int lcd_set_backlight(struct omap_dss_device *dssdev, int level)
{
	unsigned char c;
	u8 mux_pwm, enb_pwm;

	if (level > 100)
		return -1;

	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &mux_pwm, TWL_INTBR_PMBR1);
	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &enb_pwm, TWL_INTBR_GPBR1);

	if (level == 0) {
		/* disable pwm1 output and clock */
		enb_pwm = enb_pwm & 0xF5;
		/* change pwm1 pin to gpio pin */
		mux_pwm = mux_pwm & 0xCF;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					enb_pwm, TWL_INTBR_GPBR1);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					mux_pwm, TWL_INTBR_PMBR1);
		return 0;
	}

	if (!((enb_pwm & 0xA) && (mux_pwm & 0x30))) {
		/* change gpio pin to pwm1 pin */
		mux_pwm = mux_pwm | 0x30;
		/* enable pwm1 output and clock*/
		enb_pwm = enb_pwm | 0x0A;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					mux_pwm, TWL_INTBR_PMBR1);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					enb_pwm, TWL_INTBR_GPBR1);
	}

	c = ((50 * (100 - level)) / 100) + 1;
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, 0x7F, TWL_LED_PWMOFF);
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, c, TWL_LED_PWMON);

	return 0;

}


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

	return;
}

static int egf_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(DISPLAY_EN_GPIO, 1);
	gpio_set_value(LCD_3V3_ENABLE_GPIO, 1);

	return 0;
}

static void egf_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(DISPLAY_EN_GPIO, 0);
	gpio_set_value(LCD_3V3_ENABLE_GPIO, 0);

	return;
}

static struct panel_generic_dpi_data dvi_panel = {
	.name = "generic",
	.platform_enable = egf_enable_dvi,
	.platform_disable = egf_disable_dvi,
};

static struct omap_dss_device egf_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_dpi_panel",
	.data = &dvi_panel,
	.phy.dpi.data_lines = 24,
	.reset_gpio=-EINVAL,
	//.reset_gpio=213,
};

static struct omap_dss_device egf_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "egf_blc1089_ls_panel",
	.phy.dpi.data_lines = 24,
	.platform_enable = egf_enable_lcd,
	.platform_disable = egf_disable_lcd,
	//.reset_gpio=213,
	.max_backlight_level=100,
	.set_backlight=lcd_set_backlight,
	.reset_gpio=-EINVAL,
};

static struct omap_dss_device egf_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *egf_dss_devices[] = {
	&egf_lcd_device,
	&egf_dvi_device,
	&egf_tv_device,

};

static struct omap_dss_board_info egf_dss_data = {
	.num_devices = ARRAY_SIZE(egf_dss_devices),
	.devices = egf_dss_devices,
	.default_device = &egf_lcd_device,
};

/* Funzione di inizializzazione SPI */
static void __init egf_spi_init(void)
{
  printk("Registering Gpio SPI\n");
  spi_register_board_info(egf_gpio_spi, ARRAY_SIZE(egf_gpio_spi));
}


static void __init egf_display_init(void)
{
	int r;

	r = gpio_request_one(egf_dvi_device.reset_gpio, GPIOF_OUT_INIT_LOW,
			     "DVI reset");

	r = gpio_request_one(DISPLAY_EN_GPIO, GPIOF_OUT_INIT_HIGH,
			     "Display Enable");
	if (r < 0)
		printk(KERN_ERR "Unable to get Display Enable GPIO\n");

	r = gpio_request_one(LCD_3V3_ENABLE_GPIO, GPIOF_OUT_INIT_HIGH,
			     "LCD 3V3 Enable");
	if (r < 0)
		printk(KERN_ERR "Unable to get LCD 3V3 Enable GPIO\n");

}

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
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

static struct regulator_consumer_supply egf_vmmc1_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

static struct regulator_consumer_supply egf_vmmc2_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1")
};

static struct regulator_consumer_supply egf_vsim_supply[] = {
	REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.0"),
};

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
	.num_consumer_supplies	= ARRAY_SIZE(egf_vmmc1_supply),
	.consumer_supplies	= egf_vmmc1_supply,
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
	.num_consumer_supplies	= ARRAY_SIZE(egf_vmmc2_supply),
	.consumer_supplies	= egf_vmmc2_supply,
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
	.num_consumer_supplies	= ARRAY_SIZE(egf_vsim_supply),
	.consumer_supplies	= egf_vsim_supply,
};

static struct twl4030_platform_data egf_twldata = {
	/* platform_data for children goes here */
	.gpio		= &egf_gpio_data,
	.vmmc1		= &egf_vmmc1,
	.vmmc2		= &egf_vmmc2,
	.vsim		= &egf_vsim,
};

static struct at24_platform_data at24c64 = {
     .byte_len       = SZ_64K / 8,
     .flags			 = AT24_FLAG_ADDR16,
     .page_size      = 32,
};


static struct i2c_board_info __initdata egf_i2c_eeprom[] = {
       {
               I2C_BOARD_INFO("24c64", EEPROM_I2C_ADDR),
               .platform_data  = &at24c64,
       },
};
static int __init omap3_egf_i2c_init(void)
{

	omap3_pmic_get_config(&egf_twldata,
			TWL_COMMON_PDATA_USB | TWL_COMMON_PDATA_AUDIO,
			TWL_COMMON_REGULATOR_VDAC | TWL_COMMON_REGULATOR_VPLL2);
	egf_twldata.vpll2->constraints.name = "VDVI";
	omap3_pmic_init("twl4030", &egf_twldata);
	omap_register_i2c_bus(2, 100, egf_i2c_eeprom, ARRAY_SIZE(egf_i2c_eeprom));
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}



static void __init omap3_egf_init_early(void)
{
	omap2_init_common_infrastructure();
}

static void __init omap3_egf_init_irq(void)
{
	omap3_init_irq();
}

static struct platform_device *omap3_egf_devices[] __initdata = {

};

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {

	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 16,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init egf_opp_init(void)
{
	int r = 0;

	/* Initialize the omap3 opp table */
	if (omap3_opp_init()) {
		pr_err("%s: opp default init failed\n", __func__);
		return;
	}

	/* Custom OPP enabled for all xM versions */
	if (cpu_is_omap3630()) {
		struct device *mpu_dev, *iva_dev;

		mpu_dev = omap2_get_mpuss_device();
		iva_dev = omap2_get_iva_device();

		if (!mpu_dev || !iva_dev) {
			pr_err("%s: Aiee.. no mpu/dsp devices? %p %p\n",
				__func__, mpu_dev, iva_dev);
			return;
		}
		/* Enable MPU 1GHz and lower opps */
		r = opp_enable(mpu_dev, 800000000);
		/* TODO: MPU 1GHz needs SR and ABB */

		/* Enable IVA 800MHz and lower opps */
		r |= opp_enable(iva_dev, 660000000);
		/* TODO: DSP 800MHz needs SR and ABB */
		if (r) {
			pr_err("%s: failed to enable higher opp %d\n",
				__func__, r);
			/*
			 * Cleanup - disable the higher freqs - we dont care
			 * about the results
			 */
			opp_disable(mpu_dev, 800000000);
			opp_disable(iva_dev, 660000000);
		}
	}
	return;
}

static void __init omap3_egf_init(void)
{
	omap3_egf_i2c_init();
	platform_add_devices(omap3_egf_devices,
			ARRAY_SIZE(omap3_egf_devices));
	egf_spi_init();
	omap_display_init(&egf_dss_data);
	omap_serial_init();
	omap_sdrc_init(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);

	usb_musb_init(NULL);
	usbhs_init(&usbhs_bdata);
	egf_display_init();
#if defined(CONFIG_TOUCHSCREEN_SX8652)
	egf_ts_init();
#endif
	egf_opp_init();
}

MACHINE_START(OMAP3_EGF, "OMAP3 elettronicaGF SOM")
	/* Maintainer: Andrea Collamati <andrea.collamati@elettronicagf.it> */
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3_egf_init_early,
	.init_irq	= omap3_egf_init_irq,
	.init_machine	= omap3_egf_init,
	.timer		= &omap3_secure_timer,
MACHINE_END
