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
#include "mux.h"
#include "hsmmc.h"
#include "pm.h"
#include "common-board-devices.h"
#include "sdram-micron-mt46h32m32lf-6.h"

#define OMAP3_EGF_DISPLAY_ENABLE_GPIO			140
#define OMAP3_EGF_COR_nPD_TFP410 				142
#define OMAP3_EGF_COR_RESET_TFP410 			143
#define OMAP3_EGF_I2C3_SCL_GPIO_184 			184
#define OMAP3_EGF_I2C3_SDA_GPIO_185 			185

#include <linux/i2c/at24.h>
#define EEPROM_ON_MODULE_I2C_ADDR 0x50
#define EEPROM_ON_BOARD_I2C_ADDR  0x56

#define OMAP3_EGF_SERIAL_BUSY	12

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
#include <linux/smsc911x.h>
#define OMAP3EGF_ETHR_GPIO_IRQ	176
#define OMAP3EGF_SMSC911X_CS	5
#define OMAP3EGF_ETHR_GPIO_RST	64

#include <plat/gpmc-smsc911x.h>

static struct omap_smsc911x_platform_data smsc911x_cfg = {
	.cs             = OMAP3EGF_SMSC911X_CS,
	.gpio_irq       = OMAP3EGF_ETHR_GPIO_IRQ,
	.gpio_reset     = -EINVAL,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
};

static inline void __init egf_init_smsc911x(void)
{
	struct clk *l3ck;
	unsigned int rate;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	smsc911x_cfg.gpio_reset = OMAP3EGF_ETHR_GPIO_RST;

	gpmc_smsc911x_init(&smsc911x_cfg);
}

#endif





static int egf_enable_dvi(struct omap_dss_device *dssdev)
{
//	if (gpio_is_valid(dssdev->reset_gpio))
//		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void egf_disable_dvi(struct omap_dss_device *dssdev)
{
//	if (gpio_is_valid(dssdev->reset_gpio))
//		gpio_set_value(dssdev->reset_gpio, 0);

	return;
}


static struct panel_generic_dpi_data dvi_panel = {
	.name = "mbugrf-1",
	.platform_enable = egf_enable_dvi,
	.platform_disable = egf_disable_dvi,
};

static struct omap_dss_device egf_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.clocks	= {
			.dispc	= {
				.dispc_fclk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
		},
	.driver_name = "generic_dpi_panel",
	.data = &dvi_panel,
	.phy.dpi.data_lines = 24,
	.reset_gpio = OMAP3_EGF_DISPLAY_ENABLE_GPIO,
};


static struct omap_dss_device *egf_dss_devices[] = {
	&egf_dvi_device,

};

static struct omap_dss_board_info egf_dss_data = {
	.num_devices = ARRAY_SIZE(egf_dss_devices),
	.devices = egf_dss_devices,
	.default_device = &egf_dvi_device,
};


static void __init egf_display_init(void)
{
	int r;

	r = gpio_request_one(OMAP3_EGF_DISPLAY_ENABLE_GPIO, GPIOF_OUT_INIT_HIGH, "Display Enable");
	if (r < 0)
		printk(KERN_ERR "Unable to get Display Enable GPIO\n");

	r = gpio_request_one(OMAP3_EGF_COR_nPD_TFP410, GPIOF_OUT_INIT_HIGH, "TFP410 nPD");
	if (r < 0)
		printk(KERN_ERR "Unable to get OMAP3_EGF_COR_nPD_TFP410\n");

	r = gpio_request_one(OMAP3_EGF_COR_RESET_TFP410, GPIOF_OUT_INIT_LOW,"TFP410 RESET");
	if (r < 0)
		printk(KERN_ERR "Unable to get OMAP3_EGF_COR_nRESET_TFP410\n");

	r = gpio_request_one(OMAP3_EGF_I2C3_SCL_GPIO_184, GPIOF_OUT_INIT_HIGH,"I2C3_SCL");
	if (r < 0)
		printk(KERN_ERR "Unable to get OMAP3_EGF_I2C3_SCL_GPIO_184\n");

	r = gpio_request_one(OMAP3_EGF_I2C3_SDA_GPIO_185, GPIOF_OUT_INIT_LOW, "I2C3_SDA");
	if (r < 0)
		printk(KERN_ERR "Unable to get OMAP3_EGF_I2C3_SDA_GPIO_185\n");







}

static const struct gpio_led egf_leds[] __initconst = {
	{
		.name = "evb_led0",
//		.default_trigger = "default-on",
		.gpio = OMAP3_EGF_SERIAL_BUSY,
	},
};

static const struct gpio_led_platform_data egf_leds_data __initconst = {
	.leds = egf_leds,
	.num_leds = ARRAY_SIZE(egf_leds),
};


static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
	},
/*
	{
    	.mmc		= 2,
    	.caps		= MMC_CAP_4_BIT_DATA,
    	.gpio_wp	= -EINVAL,
		.ext_clock	= 1,
		.transceiver	= true,

  },
*/
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
//static struct at24_platform_data at24c04 = {
//     .byte_len       = SZ_4K / 8,
//     .page_size      = 16,
//};



static int __init omap3_egf_i2c_init(void)
{

	omap3_pmic_get_config(&egf_twldata,
			TWL_COMMON_PDATA_USB | TWL_COMMON_PDATA_AUDIO,
			TWL_COMMON_REGULATOR_VDAC | TWL_COMMON_REGULATOR_VPLL2);
	egf_twldata.vpll2->constraints.name = "VDVI";
	omap3_pmic_init("twl4030", &egf_twldata);
	omap_register_i2c_bus(2, 400, egf_i2c_eeprom_on_module, ARRAY_SIZE(egf_i2c_eeprom_on_module));
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
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

		mpu_dev = omap_device_get_by_hwmod_name("mpu");
		iva_dev = omap_device_get_by_hwmod_name("iva");

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
	egf_display_init();
	omap_display_init(&egf_dss_data);
	omap_serial_init();
	omap_sdrc_init(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);

//	usb_musb_init(NULL);
	usbhs_init(&usbhs_bdata);
	egf_init_smsc911x();
	gpio_led_register_device(-1, &egf_leds_data);
	egf_opp_init();
}

MACHINE_START(OMAP3_EGF, "OMAP3 elettronicaGF SOM")
	/* Maintainer: Andrea Collamati <andrea.collamati@elettronicagf.it> */
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3_init_early,
	.init_irq	= omap3_init_irq,
	.init_machine	= omap3_egf_init,
	.timer		= &omap3_secure_timer,
MACHINE_END
