/*
 * Support for egf_blc1097 LCD Panel used on EGF Board 396
 *
 * Copyright (C) 2012 Elettronica GF
 *
 * Original Driver Author: Imre Deak <imre.deak@nokia.com>
 * Based on panel-generic.c by Tomi Valkeinen <tomi.valkeinen@nokia.com>
 * Adapted to new DSS2 framework: Roger Quadros <roger.quadros@nokia.com>
 * Adapted to new BLC1097: Roger Quadros <andrea.collamati@elettronicagf.it>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <plat/display.h>

#define MIPID_CMD_RDDPM			0x0A
#define MIPID_CMD_RDDCOLMOD		0x0C
#define MIPID_CMD_SLEEP_IN		0x10
#define MIPID_CMD_SLEEP_OUT		0x11
#define MIPID_CMD_DISP_OFF		0x28
#define MIPID_CMD_DISP_ON		0x29
#define MIPID_CMD_WRITE_DISP_BRIGHTNESS	0x51
#define MIPID_CMD_READ_DISP_BRIGHTNESS	0x52
#define MIPID_CMD_WRITE_CTRL_DISP	0x53

#define CTRL_DISP_BRIGHTNESS_CTRL_ON	(1 << 5)
#define CTRL_DISP_AMBIENT_LIGHT_CTRL_ON	(1 << 4)
#define CTRL_DISP_BACKLIGHT_ON		(1 << 2)
#define CTRL_DISP_AUTO_BRIGHTNESS_ON	(1 << 1)

#define MIPID_CMD_READ_CTRL_DISP	0x54
#define MIPID_CMD_WRITE_CABC		0x55
#define MIPID_CMD_READ_CABC		0x56

#define MIPID_VER_LPH8923		3
#define MIPID_VER_LS041Y3		4
#define MIPID_VER_L4F00311		8
#define MIPID_VER_egf_blc1097		9

struct egf_blc1097_device {
	char		*name;
	int		enabled;
	int		model;
	int		revision;
	u8		display_id[3];
	unsigned long	hw_guard_end;		/* next value of jiffies
						   when we can issue the
						   next sleep in/out command */
	unsigned long	hw_guard_wait;		/* max guard time in jiffies */

	struct spi_device	*spi;
	struct mutex		mutex;

	struct omap_dss_device	*dssdev;
	struct backlight_device *bl_dev;
};

static struct egf_blc1097_device egf_panel_dev;
static int egf_blc1097_bl_update_status(struct backlight_device *dev);

/*--------------------MIPID interface-----------------------------*/

static void egf_blc1097_transfer(struct egf_blc1097_device *md, int cmd,
			      const u8 *wbuf, int wlen, u8 *rbuf, int rlen)
{
	struct spi_message	m;
	struct spi_transfer	*x, xfer[5];
	int			r;

	BUG_ON(md->spi == NULL);

	spi_message_init(&m);

	memset(xfer, 0, sizeof(xfer));
	x = &xfer[0];

	cmd &=  0xff;
	cmd |= 0xAA00;
	x->tx_buf = &cmd;
	x->bits_per_word = 9;
	x->len = 2;

	if (rlen >= 1 && wlen == 0) {
		/*
		 * Between the command and the response data there is a
		 * dummy clock cycle. Add an extra bit after the command
		 * word to account for this.
		 */
		x->bits_per_word = 10;
		cmd <<= 1;
	}
	spi_message_add_tail(x, &m);

	if (wlen) {
		x++;
		x->tx_buf = wbuf;
		x->len = wlen;
		x->bits_per_word = 9;
		spi_message_add_tail(x, &m);
	}

	if (rlen) {
		x++;
		x->rx_buf	= rbuf;
		x->len		= rlen;
		spi_message_add_tail(x, &m);
	}

	r = spi_sync(md->spi, &m);
	if (r < 0)
		printk(KERN_INFO  "spi_sync %d\n", r);
}

static inline void egf_blc1097_cmd(struct egf_blc1097_device *md, int cmd)
{
	egf_blc1097_transfer(md, cmd, NULL, 0, NULL, 0);
}

static inline void egf_blc1097_write(struct egf_blc1097_device *md,
			       int reg, const u8 *buf, int len)
{
	egf_blc1097_transfer(md, reg, buf, len, NULL, 0);
}

static inline void egf_blc1097_read(struct egf_blc1097_device *md,
			      int reg, u8 *buf, int len)
{
	egf_blc1097_transfer(md, reg, NULL, 0, buf, len);
}

static void hw_guard_start(struct egf_blc1097_device *md, int guard_msec)
{
	md->hw_guard_wait = msecs_to_jiffies(guard_msec);
	md->hw_guard_end = jiffies + md->hw_guard_wait;
}

static void hw_guard_wait(struct egf_blc1097_device *md)
{
	unsigned long wait = md->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= md->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

/*----------------------MIPID wrappers----------------------------*/

static void set_sleep_mode(struct egf_blc1097_device *md, int on)
{
	int cmd;

	if (on)
		cmd = MIPID_CMD_SLEEP_IN;
	else
		cmd = MIPID_CMD_SLEEP_OUT;
	/*
	 * We have to keep 120msec between sleep in/out commands.
	 * (8.2.15, 8.2.16).
	 */
	hw_guard_wait(md);
	egf_blc1097_cmd(md, cmd);
	hw_guard_start(md, 120);
}

static void set_display_state(struct egf_blc1097_device *md, int enabled)
{
	int cmd = enabled ? MIPID_CMD_DISP_ON : MIPID_CMD_DISP_OFF;

	egf_blc1097_cmd(md, cmd);
}



//static int panel_detect(struct egf_blc1097_device *md)
//{
//	egf_blc1097_read(md, MIPID_CMD_READ_DISP_ID, md->display_id, 3);
//	printk(KERN_INFO  "MIPI display ID: %02x%02x%02x\n",
//		md->display_id[0], md->display_id[1], md->display_id[2]);
//
//	switch (md->display_id[0]) {
//	case 0x10:
//		md->model = MIPID_VER_egf_blc1097;
//		md->name = "egf_blc1097";
//		md->has_bc = 1;
//		md->has_cabc = 1;
//		break;
//	case 0x29:
//		md->model = MIPID_VER_L4F00311;
//		md->name = "l4f00311";
//		break;
//	case 0x45:
//		md->model = MIPID_VER_LPH8923;
//		md->name = "lph8923";
//		break;
//	case 0x83:
//		md->model = MIPID_VER_LS041Y3;
//		md->name = "ls041y3";
//		break;
//	default:
//		md->name = "unknown";
//		dev_err(&md->spi->dev, "invalid display ID\n");
//		return -ENODEV;
//	}
//
//	md->revision = md->display_id[1];
//
//	dev_info(&md->spi->dev, "omapfb: %s rev %02x LCD detected\n",
//			md->name, md->revision);
//
//	return 0;
//}

/*----------------------Backlight Control-------------------------*/

//static void enable_backlight_ctrl(struct egf_blc1097_device *md, int enable)
//{
//	u16 ctrl;
//
//	egf_blc1097_read(md, MIPID_CMD_READ_CTRL_DISP, (u8 *)&ctrl, 1);
//	if (enable) {
//		ctrl |= CTRL_DISP_BRIGHTNESS_CTRL_ON |
//			CTRL_DISP_BACKLIGHT_ON;
//	} else {
//		ctrl &= ~(CTRL_DISP_BRIGHTNESS_CTRL_ON |
//			  CTRL_DISP_BACKLIGHT_ON);
//	}
//
//	ctrl |= 1 << 8;
//	egf_blc1097_write(md, MIPID_CMD_WRITE_CTRL_DISP, (u8 *)&ctrl, 2);
//}


//static void egf_blc1097_set_brightness(struct egf_blc1097_device *md, int level)
//{
//	int bv;
//
//	bv = level | (1 << 8);
//	egf_blc1097_write(md, MIPID_CMD_WRITE_DISP_BRIGHTNESS, (u8 *)&bv, 2);
//
//	if (level)
//		enable_backlight_ctrl(md, 1);
//	else
//		enable_backlight_ctrl(md, 0);
//}
//
//static int egf_blc1097_get_actual_brightness(struct egf_blc1097_device *md)
//{
//	u8 bv;
//
//	egf_blc1097_read(md, MIPID_CMD_READ_DISP_BRIGHTNESS, &bv, 1);
//
//	return bv;
//}


static int egf_blc1097_bl_update_status(struct backlight_device *bl)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	struct egf_blc1097_device *md = &egf_panel_dev;
	int level;
	u8 ctrl;
	if (!dssdev->set_backlight)
		return -EINVAL;

	egf_blc1097_read(md, MIPID_CMD_RDDCOLMOD, (u8 *)&ctrl, 1);
	printk(KERN_INFO  "MIPID_CMD_RDDCOLMOD: %02x\n",ctrl);

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	return dssdev->set_backlight(dssdev, level);
}

static int egf_blc1097_bl_get_intensity(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
				bl->props.power == FB_BLANK_UNBLANK)
			return bl->props.brightness;

	return 0;
}

static const struct backlight_ops egf_blc1097_bl_ops = {
	.get_brightness = egf_blc1097_bl_get_intensity,
	.update_status  = egf_blc1097_bl_update_status,
};



static int egf_panel_get_recommended_bpp(struct omap_dss_device *dssdev)
{
	return 18;
}

static struct omap_video_timings egf_panel_timings = {
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 24000,
	.hfp		= 28,
	.hsw		= 4,
	.hbp		= 24,
	.vfp		= 3,
	.vsw		= 3,
	.vbp		= 4,
};

static int egf_panel_probe(struct omap_dss_device *dssdev)
{
	int r;
	struct egf_blc1097_device *md = &egf_panel_dev;
	struct backlight_device *bldev;
//	int max_brightness, brightness;
	struct backlight_properties props;

	printk(KERN_INFO  "%s\n", __func__);
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
					OMAP_DSS_LCD_IHS;
	/* FIXME AC bias ? */
	dssdev->panel.timings = egf_panel_timings;

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);
	/*
	 * After reset we have to wait 5 msec before the first
	 * command can be sent.
	 */
	msleep(5);

	md->enabled = 1;

//	r = panel_detect(md);
//	if (r) {
//		printk(KERN_INFO  "%s panel detect error\n", __func__);
//		if (!md->enabled && dssdev->platform_disable)
//			dssdev->platform_disable(dssdev);
//		return r;
//	}

	mutex_lock(&egf_panel_dev.mutex);
	egf_panel_dev.dssdev = dssdev;
	mutex_unlock(&egf_panel_dev.mutex);


	/*------- Backlight control --------*/

	props.fb_blank = FB_BLANK_UNBLANK;
	props.power = FB_BLANK_UNBLANK;

	bldev =  backlight_device_register("egf_blc1097_bl", &dssdev->dev, dssdev,
			&egf_blc1097_bl_ops, &props);

	md->bl_dev = bldev;

	bldev->props.fb_blank = FB_BLANK_UNBLANK;
	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.brightness = dssdev->max_backlight_level;
	r = egf_blc1097_bl_update_status(bldev);
	if (r < 0)
		printk(KERN_INFO  "failed to set lcd brightness\n");

	return 0;
}

static void egf_panel_remove(struct omap_dss_device *dssdev)
{
	struct egf_blc1097_device *md = &egf_panel_dev;

	printk(KERN_INFO  "%s\n", __func__);
	backlight_device_unregister(md->bl_dev);
	mutex_lock(&egf_panel_dev.mutex);
	egf_panel_dev.dssdev = NULL;
	mutex_unlock(&egf_panel_dev.mutex);
}

static int egf_panel_power_on(struct omap_dss_device *dssdev)
{
	struct egf_blc1097_device *md = &egf_panel_dev;
	int r;

	printk(KERN_INFO  "%s\n", __func__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	mutex_lock(&md->mutex);

	r = omapdss_dpi_display_enable(dssdev);
	if (r) {
		pr_err("%s dpi enable failed\n", __func__);
		goto fail_unlock;
	}

//	/*FIXME tweak me */
//	msleep(50);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto fail;
	}

//	if (md->enabled) {
//		printk(KERN_INFO  "panel already enabled\n");
//		mutex_unlock(&md->mutex);
//		return 0;
//	}

//	set_sleep_mode(md, 0);
//	md->enabled = 1;
//
//	/* 5msec between sleep out and the next command. (8.2.16) */
//	msleep(5);
	set_display_state(md, 1);

	mutex_unlock(&md->mutex);

	return egf_blc1097_bl_update_status(md->bl_dev);
fail:
	omapdss_dpi_display_disable(dssdev);
fail_unlock:
	mutex_unlock(&md->mutex);
	return r;
}

static void egf_panel_power_off(struct omap_dss_device *dssdev)
{
	struct egf_blc1097_device *md = &egf_panel_dev;

	printk(KERN_INFO  "%s\n", __func__);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	mutex_lock(&md->mutex);

	if (!md->enabled) {
		mutex_unlock(&md->mutex);
		return;
	}
	set_display_state(md, 0);
	set_sleep_mode(md, 1);
	md->enabled = 0;
	/*
	 * We have to provide PCLK,HS,VS signals for 2 frames (worst case
	 * ~50msec) after sending the sleep in command and asserting the
	 * reset signal. We probably could assert the reset w/o the delay
	 * but we still delay to avoid possible artifacts. (7.6.1)
	 */
	msleep(50);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* FIXME need to tweak this delay */
	msleep(100);

	omapdss_dpi_display_disable(dssdev);

	mutex_unlock(&md->mutex);
}

static int egf_panel_enable(struct omap_dss_device *dssdev)
{
	int r;

	r = egf_panel_power_on(dssdev);

	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return 0;
}

static void egf_panel_disable(struct omap_dss_device *dssdev)
{
	egf_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int egf_panel_suspend(struct omap_dss_device *dssdev)
{
	egf_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int egf_panel_resume(struct omap_dss_device *dssdev)
{
	int r;

	printk(KERN_INFO  "%s\n", __func__);
	r = egf_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return 0;
}

static void egf_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void egf_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int egf_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return 0;
}


static struct omap_dss_driver egf_panel_driver = {
	.probe		= egf_panel_probe,
	.remove		= egf_panel_remove,

	.enable		= egf_panel_enable,
	.disable	= egf_panel_disable,
	.suspend	= egf_panel_suspend,
	.resume		= egf_panel_resume,

	.set_timings	= egf_panel_set_timings,
	.get_timings	= egf_panel_get_timings,
	.check_timings	= egf_panel_check_timings,

	.get_recommended_bpp = egf_panel_get_recommended_bpp,

	.driver         = {
		.name   = "panel-egf_blc1097",
		.owner  = THIS_MODULE,
	},
};

/*--------------------SPI probe-------------------------*/

static int egf_blc1097_spi_probe(struct spi_device *spi)
{
	struct egf_blc1097_device *md = &egf_panel_dev;

	printk(KERN_INFO  "%s\n", __func__);

	spi->mode = SPI_MODE_3;
	md->spi = spi;
	mutex_init(&md->mutex);
	dev_set_drvdata(&spi->dev, md);

	omap_dss_register_driver(&egf_panel_driver);

	return 0;
}

static int egf_blc1097_spi_remove(struct spi_device *spi)
{
//	struct egf_blc1097_device *md = dev_get_drvdata(&spi->dev);

	printk(KERN_INFO  "%s\n", __func__);
	omap_dss_unregister_driver(&egf_panel_driver);

	return 0;
}

static struct spi_driver egf_blc1097_spi_driver = {
	.driver = {
		.name	= "egf_blc1097",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= egf_blc1097_spi_probe,
	.remove	= __devexit_p(egf_blc1097_spi_remove),
};

static int __init egf_blc1097_init(void)
{
	return spi_register_driver(&egf_blc1097_spi_driver);
}

static void __exit egf_blc1097_exit(void)
{
	spi_unregister_driver(&egf_blc1097_spi_driver);
}

module_init(egf_blc1097_init);
module_exit(egf_blc1097_exit);

MODULE_AUTHOR("Andrea Collamati");
MODULE_DESCRIPTION("egf_blc1097 LCD Driver");
MODULE_LICENSE("GPL");
