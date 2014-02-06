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
#include <asm/setup.h>
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

#include <linux/i2c/sx150x.h>

#define OMAP3_EGF_DISPLAY_ENABLE_GPIO			213
#define OMAP3_EGF_LCD_3V3_ENABLE_GPIO 			2

/* EEPROM */
#include <linux/i2c/at24.h>
#define EEPROM_ON_MODULE_I2C_ADDR 0x50
#define EEPROM_ON_BOARD_I2C_ADDR  0x54


/* TVP5151 */
#ifdef CONFIG_VIDEO_TVP515X
#define TVP5150_I2C_BUS_NUM		2
#define OMAP3_EGF_TVP5150_NRESET_GPIO		164
#define OMAP3_EGF_TVP5150_ENABLE_GPIO		163

#include <media/tvp515x.h>
#include <../drivers/media/video/isp/isp.h>
#include "devices.h"
#endif

#ifdef CONFIG_VIDEO_TVP515X_SOM336_INPUT_SELECTOR
#define OMAP3_EGF_TVP5150_INPUT_SEL_GPIO	138
#endif

/* TOUCHSCREEN */
#ifdef CONFIG_TOUCHSCREEN_SX8652
#define	OMAP3_EGF_TS_GPIO 114
#include <linux/spi/sx8652.h>
#endif


#define PWR_P1_SW_EVENTS	0x10
#define PWR_DEVOFF	(1<<0)

/* SOM REVISIONS */
#define REV_STR_TO_REV_CODE(REV_STRING) \
	(\
	(((REV_STRING[3]-'0')*1000 + (REV_STRING[4]-'0')*100+(REV_STRING[5]-'0')*10 + (REV_STRING[6]-'0')) << 16)|\
	((REV_STRING[8]-'A') << 8)|\
	((REV_STRING[9]-'0')*10 + (REV_STRING[10]-'0'))\
	)

#define SOM_REV_CODE(REV1,REV2,REV3)\
	((REV1<<16) | ((REV2-'A') << 8) |  REV3)

#define REV_NOT_PROGRAMMED  SOM_REV_CODE(((0xFF-'0')*1000 + (0xFF-'0')*100+(0xFF-'0')*10 + 0xff-'0'),'A',0xFF)
#define REV_336_A01  SOM_REV_CODE(336,'A',1)
#define REV_336_B01  SOM_REV_CODE(336,'B',1)
#define REV_336_C01  SOM_REV_CODE(336,'C',1)
#define REV_336_D01  SOM_REV_CODE(336,'D',1)
#define REV_336_E01  SOM_REV_CODE(336,'E',1)
#define REV_336_F01  SOM_REV_CODE(336,'F',1)
#define REV_336_F02  SOM_REV_CODE(336,'F',2)
#define SOM_REVISION_LEN  12  /* termination character included. ex: JSC0336_A02*/


#define GPIO_DEBUG_LED_RED		264
#define GPIO_DEBUG_LED_YELLOW	265
#define GPIO_DEBUG_LED_BLUE		266
#define GPIO_LED_1				272
#define GPIO_LED_2				273
#define GPIO_LED_3				274
#define GPIO_LED_4				275
#define GPIO_LED_5				276

 struct egf_som {
 	__u32 rev_code;
 	int has_tvp5150;
 };
 struct egf_som the_som;


 static int load_som_revision(int som_rev_code)
 {
 	while (1) {
 		the_som.rev_code = som_rev_code;
 		switch (the_som.rev_code) {
 		case REV_336_A01:
 		case REV_336_B01:
 		case REV_336_C01:
 		case REV_336_D01:
 		case REV_336_E01:
 			the_som.has_tvp5150 = 1;
 			return 0;
 		case REV_336_F01:
 			the_som.has_tvp5150 = 0;
 			return 0;
 		case REV_336_F02:
 			the_som.has_tvp5150 = 0;
 			return 0;
 		case REV_NOT_PROGRAMMED:
 		default:
 			the_som.has_tvp5150 = 0;
 			printk(KERN_ERR "somrev not supported!\n");
 		}
 	}
 	return 0;
 }
 static void __init fixup_egf_board(struct machine_desc *desc,
 		struct tag *tags, char **cmdline, struct meminfo *mi)
 {
	struct tag *t;
	char *str;
	char *str_som_rev;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "somrev=");
			if (str != NULL) {
				str_som_rev = str + 7;
				printk(KERN_INFO "SOM REV:%s\n",str_som_rev);
				load_som_revision(REV_STR_TO_REV_CODE(str_som_rev));
			}else
				printk(KERN_ERR "Somrev boot param missing!\n");
			break;
		}
	}

 }

static struct gpio_led gpio_leds[] = {
 		{
 			 .name = "DEBUG-LED-BLUE",
 			 .default_trigger = "heartbeat",
 			 .active_low = 0,
 			 .gpio = GPIO_DEBUG_LED_BLUE,
 		},
 		{
 			 .name = "DEBUG-LED-YELLOW",
 			 .default_trigger = "none",
 			 .active_low = 0,
 			 .gpio = GPIO_DEBUG_LED_YELLOW,
 		},
 		{
 			 .name = "DEBUG-LED-RED",
 			 .default_trigger = "none",
 			 .active_low = 0,
 			 .gpio = GPIO_DEBUG_LED_RED,
 		},
 		{
 			 .name = "LED-1",
 			 .default_trigger = "none",
 			 .active_low = 1,
 			 .gpio = GPIO_LED_1,
 		},
 		{
 			 .name = "LED-2",
 			 .default_trigger = "none",
 			 .active_low = 1,
 			 .gpio = GPIO_LED_2,
 		},
 		{
 			 .name = "LED-3",
 			 .default_trigger = "none",
 			 .active_low = 1,
 			 .gpio = GPIO_LED_3,
 		},
 		{
 			 .name = "LED-4",
 			 .default_trigger = "none",
 			 .active_low = 1,
 			 .gpio = GPIO_LED_4,
 		},
 		{
 			 .name = "LED-5",
 			 .default_trigger = "none",
 			 .active_low = 1,
 			 .gpio = GPIO_LED_5,
 		},
};

static struct gpio_led_platform_data gpio_led_info = {
		.leds = gpio_leds,
		.num_leds = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
			.platform_data = &gpio_led_info,
	},
};


static void twl4030_poweroff(void)
{
	u8 val;
	int err;
	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &val,
				  PWR_P1_SW_EVENTS);
	if (err) {
		printk(KERN_WARNING "I2C error %d while reading TWL4030"
					"PM_MASTER P1_SW_EVENTS\n", err);
		return ;
	}

	val |= PWR_DEVOFF;

	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, val,
				   PWR_P1_SW_EVENTS);

	if (err) {
		printk(KERN_WARNING "I2C error %d while writing TWL4030"
					"PM_MASTER P1_SW_EVENTS\n", err);
		return ;
	}

	return;
}

static int __init twl4030_poweroff_init(void)
{
	pm_power_off = twl4030_poweroff;

	return 0;
}



#ifdef CONFIG_VIDEO_TVP515X
static int egf_tvp515x_s_power(struct v4l2_subdev *subdev, u32 on)
{
	gpio_set_value(OMAP3_EGF_TVP5150_ENABLE_GPIO, on);
	return 0;
}
/* TVP5150: Video Decoder */
static struct tvp515x_platform_data egf_tvp515x_platform_data = {
	.s_power		= egf_tvp515x_s_power,
};
static struct i2c_board_info egf_camera_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tvp5151", 0x5C),
		.platform_data	= &egf_tvp515x_platform_data,
	},
};


static struct isp_subdev_i2c_board_info egf_tvp5150_subdevs[] = {
	{
		.board_info	= &egf_camera_i2c_devices[0],
		.i2c_adapter_id	= TVP5150_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};

static struct isp_v4l2_subdevs_group egf_camera_subdevs[] = {
	{
		.subdevs	= egf_tvp5150_subdevs,
		.interface	= ISP_INTERFACE_PARALLEL,
		.bus		= {
			.parallel	= {
				.width			= 8,
				.data_lane_shift	= 0,
				.clk_pol		= 0,
				.hdpol			= 0,
				.vdpol			= 1,
				.fldmode		= 1,
				.bridge			= 0,
				.is_bt656		= 1,
			},
		},
	},
	{ NULL, 0 },
};

static struct isp_platform_data egf_isp_platform_data = {
	.subdevs = egf_camera_subdevs,
};

static int __init egf_cam_init(void)
{
	int ret = 0;
/*
#ifdef CONFIG_VIDEO_TVP515X_SOM336_INPUT_SELECTOR
	omap_mux_init_gpio(OMAP3_EGF_TVP5150_INPUT_SEL_GPIO, OMAP_PIN_OUTPUT);
#endif
	omap_mux_init_gpio(OMAP3_EGF_TVP5150_NRESET_GPIO, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(OMAP3_EGF_TVP5150_ENABLE_GPIO, OMAP_PIN_INPUT_PULLUP);
*/


	/* Force tvp5150 nreset to 1 */
	ret = gpio_request(OMAP3_EGF_TVP5150_NRESET_GPIO, "tvp5150_nreset");
	if (ret) {
		printk(KERN_ERR "failed to get tvp5150_nreset\n");
		goto err_1;
	}
	gpio_direction_output(OMAP3_EGF_TVP5150_NRESET_GPIO, 1);

	/* Initialize tvp5150 enable to 0 */
	ret = gpio_request(OMAP3_EGF_TVP5150_ENABLE_GPIO, "tvp5150_enable");
	if (ret) {
		printk(KERN_ERR "failed to get tvp5150_enable\n");
		goto err_2;
	}
	/* TVP5150 should be on or is not detected at probing level */
	gpio_direction_output(OMAP3_EGF_TVP5150_ENABLE_GPIO, 1);

	omap3_init_camera(&egf_isp_platform_data);
	printk(KERN_INFO "egf camera init done successfully...\n");
	return 0;
err_2:
	gpio_free(OMAP3_EGF_TVP5150_NRESET_GPIO);
err_1:
	return ret;
}



#endif


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

	if (level == 100)
		c = 0x7F;
	else
		c = ((128 * (100 - level)) / 100) + 1;

	twl_i2c_write_u8(TWL4030_MODULE_PWM1, 0x7F, TWL_LED_PWMOFF);
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, c, TWL_LED_PWMON);

	return 0;

}

static int egf_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(OMAP3_EGF_DISPLAY_ENABLE_GPIO, 1);
	gpio_set_value(OMAP3_EGF_LCD_3V3_ENABLE_GPIO, 1);

	return 0;
}

static void egf_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(OMAP3_EGF_DISPLAY_ENABLE_GPIO, 0);
	gpio_set_value(OMAP3_EGF_LCD_3V3_ENABLE_GPIO, 0);

	return;
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

static struct omap_dss_device egf_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "egf_tx20d26vm0aaa_ls_panel",
	.phy.dpi.data_lines = 24,
	.platform_enable = egf_enable_lcd,
	.platform_disable = egf_disable_lcd,
	//.reset_gpio=213,
	.max_backlight_level = 100,
	.set_backlight = lcd_set_backlight,
	.reset_gpio = OMAP3_EGF_DISPLAY_ENABLE_GPIO,
};

static struct omap_dss_device egf_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
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

	r = gpio_request(OMAP3_EGF_LCD_3V3_ENABLE_GPIO, "LCD 3V3 Enable");
		if (r < 0) {
			printk(KERN_ERR "Unable to get LCD 3V3 Enable GPIO\n");
			return;
		}

	gpio_direction_output(OMAP3_EGF_LCD_3V3_ENABLE_GPIO, 0);

}

/* EEPROM  */

/* EEprom on SOM336 */
static struct at24_platform_data at24c64 = {
     .byte_len       = SZ_64K / 8,
     .flags			 = AT24_FLAG_ADDR16,
     .page_size      = 32,
};

static struct sx150x_platform_data __initdata sx1509_gpio_expander_onboard_data;

static struct i2c_board_info __initdata egf_i2c2_devices[] = {
       {
               I2C_BOARD_INFO("24c64", EEPROM_ON_MODULE_I2C_ADDR),
               .platform_data  = &at24c64,
       },
};


static struct i2c_board_info __initdata egf_i2c3_devices[] = {
       {
               I2C_BOARD_INFO("24c128", EEPROM_ON_BOARD_I2C_ADDR),
       },
       {
               I2C_BOARD_INFO("sx1509q", 0x70),
               .platform_data = &sx1509_gpio_expander_onboard_data,
       },
       {
    		   I2C_BOARD_INFO("tmp102", 0x49),
        },
};

static void __init init_gpio_expander(void)
{
	sx1509_gpio_expander_onboard_data.irq_summary = -1;
	sx1509_gpio_expander_onboard_data.gpio_base = 264;
}


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
	{
		.mmc		= 3,
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

/*
static struct regulator_consumer_supply egf_vsim_supply = {
	.supply			= "vmmc_aux",
};
*/


static struct gpio_led gpio_leds[];

static int egf_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	mmc[2].gpio_cd = gpio + 1;
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
/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA)
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
};*/

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
static struct twl4030_madc_platform_data egf_madc_data = {
 .irq_line = 1,
};

static uint32_t board_keymap[] = {
	KEY(0, 0, KEY_F1),
	KEY(0, 1, KEY_UP),
	KEY(0, 2, KEY_ENTER),

	KEY(1, 0, KEY_F2),
	KEY(1, 1, KEY_DOWN),
	KEY(1, 3, KEY_VOLUMEUP),

	KEY(2, 0, KEY_F3),
	KEY(2, 1, KEY_LEFT),
	KEY(2, 3, KEY_VOLUMEDOWN),

	KEY(3, 0, KEY_F4),
	KEY(3, 1, KEY_RIGHT),
	KEY(3, 3, KEY_POWER),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data omap3egf_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 4,
	.rep		= 1,
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
//	.vsim		= &egf_vsim,
	.vdac		= &egf_vdac,
	.vpll2		= &egf_vpll2,
	.madc		= &egf_madc_data,
	.keypad		= &omap3egf_kp_data,
};

static struct i2c_board_info __initdata egf_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &egf_twldata,
	},
};

static int __init omap3_egf_i2c_init(void)
{
	init_gpio_expander();
	omap_register_i2c_bus(1, 2600, egf_i2c1_devices,ARRAY_SIZE(egf_i2c1_devices));
	omap_register_i2c_bus(2, 400, egf_i2c2_devices, ARRAY_SIZE(egf_i2c2_devices));
	omap_register_i2c_bus(3, 400, egf_i2c3_devices, ARRAY_SIZE(egf_i2c3_devices));

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
	omap2_gp_clockevent_set_gptimer(12);

}

static struct platform_device *omap3_egf_devices[] __initdata = {
	&egf_dss_device,
};


static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
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
	if (cpu_is_omap3630()){
		printk("Muxing Detected 37xx\n");
		omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	}
	else{
		omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	}
	omap3_egf_spi_init();
	omap3_egf_i2c_init();
	platform_add_devices(omap3_egf_devices,
			ARRAY_SIZE(omap3_egf_devices));
	omap_serial_init();


	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);
	egf_ts_init();
	egf_display_init();
	twl4030_poweroff_init();
	platform_device_register(&leds_gpio);
#ifdef CONFIG_VIDEO_TVP515X
	if(the_som.has_tvp5150)
		egf_cam_init();
#endif
}

MACHINE_START(OMAP3_EGF, "OMAP3 EGF")
	/* Maintainer: Andrea Collamati - http://www.elettronicagf.it */
	.boot_params= 0x80000100,
	.fixup		= fixup_egf_board,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_egf_init_irq,
	.init_machine	= omap3_egf_init,
	.timer		= &omap_timer,
MACHINE_END
