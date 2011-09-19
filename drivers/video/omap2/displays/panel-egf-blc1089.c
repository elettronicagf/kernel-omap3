/*
 * LCD panel driver for egf_blc1089 LS037V7DW01
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <video/omapdss.h>

struct egf_blc1089_data {
	struct backlight_device *bl;
};

static struct omap_video_timings egf_blc1089_ls_timings = {
	.x_res = 480,
	.y_res = 272,

	.pixel_clock	= 9000,

	.hsw		= 41,
	.hfp		= 2,
	.hbp		= 2,

	.vsw		= 10,
	.vfp		= 2,
	.vbp		= 2,
};

static void egf_blc1089_ls_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void egf_blc1089_ls_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int egf_blc1089_ls_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static int egf_blc1089_ls_bl_update_status(struct backlight_device *bl)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	int level;

	if (!dssdev->set_backlight)
		return -EINVAL;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	return dssdev->set_backlight(dssdev, level);
}

static int egf_blc1089_ls_bl_get_brightness(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}

static const struct backlight_ops egf_blc1089_ls_bl_ops = {
	.get_brightness = egf_blc1089_ls_bl_get_brightness,
	.update_status  = egf_blc1089_ls_bl_update_status,
};



static int egf_blc1089_ls_panel_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	struct egf_blc1089_data *sd;
	int r;

	dssdev->panel.config = OMAP_DSS_LCD_TFT|OMAP_DSS_LCD_IPC;
	dssdev->panel.acb = 0x0;
	dssdev->panel.timings = egf_blc1089_ls_timings;

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, sd);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = dssdev->max_backlight_level;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register("egf_blc1089-ls", &dssdev->dev, dssdev,
			&egf_blc1089_ls_bl_ops, &props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		kfree(sd);
		return r;
	}
	sd->bl = bl;

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = dssdev->max_backlight_level;
	r = egf_blc1089_ls_bl_update_status(bl);
	if (r < 0)
		dev_err(&dssdev->dev, "failed to set lcd brightness\n");

	return 0;
}

static void __exit egf_blc1089_ls_panel_remove(struct omap_dss_device *dssdev)
{
	struct egf_blc1089_data *sd = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bl = sd->bl;

	bl->props.power = FB_BLANK_POWERDOWN;
	egf_blc1089_ls_bl_update_status(bl);
	backlight_device_unregister(bl);

	kfree(sd);
}

static int egf_blc1089_ls_power_on(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void egf_blc1089_ls_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);

	omapdss_dpi_display_disable(dssdev);
}

static int egf_blc1089_ls_panel_enable(struct omap_dss_device *dssdev)
{
	int r;
	r = egf_blc1089_ls_power_on(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return r;
}

static void egf_blc1089_ls_panel_disable(struct omap_dss_device *dssdev)
{
	egf_blc1089_ls_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int egf_blc1089_ls_panel_suspend(struct omap_dss_device *dssdev)
{
	egf_blc1089_ls_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int egf_blc1089_ls_panel_resume(struct omap_dss_device *dssdev)
{
	int r;
	r = egf_blc1089_ls_power_on(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return r;
}

static struct omap_dss_driver egf_blc1089_ls_driver = {
	.probe		= egf_blc1089_ls_panel_probe,
	.remove		= __exit_p(egf_blc1089_ls_panel_remove),

	.enable		= egf_blc1089_ls_panel_enable,
	.disable	= egf_blc1089_ls_panel_disable,
	.suspend	= egf_blc1089_ls_panel_suspend,
	.resume		= egf_blc1089_ls_panel_resume,
	.set_timings	= egf_blc1089_ls_panel_set_timings,
	.get_timings	= egf_blc1089_ls_panel_get_timings,
	.check_timings	= egf_blc1089_ls_panel_check_timings,
	.driver         = {
		.name   = "egf_blc1089_ls_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init egf_blc1089_ls_panel_drv_init(void)
{
	return omap_dss_register_driver(&egf_blc1089_ls_driver);
}

static void __exit egf_blc1089_ls_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&egf_blc1089_ls_driver);
}

module_init(egf_blc1089_ls_panel_drv_init);
module_exit(egf_blc1089_ls_panel_drv_exit);
MODULE_LICENSE("GPL");
