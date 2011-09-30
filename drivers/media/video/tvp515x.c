/*
 * drivers/media/video/tvp515x.c
 *
 * TI TVP5150/51 decoder driver
 *
 * Using code from drivers/media/video/tvp514x.c
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>

#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/tvp515x.h>

#include "tvp515x_regs.h"

/* GPIO for input selector on SOM336 Module */
#ifdef CONFIG_VIDEO_TVP515X_SOM336_INPUT_SELECTOR
#include <linux/gpio.h>
#define OMAP3_EGF_TVP5150_INPUT_SEL_GPIO	138
#endif
/* Module Name */
#define TVP515x_MODULE_NAME		"tvp515x"

/* Private macros for TVP */
#define I2C_RETRY_COUNT                 (5)
#define LOCK_RETRY_COUNT                (5)
#define LOCK_RETRY_DELAY                (200)

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("tvp515x linux decoder driver");
MODULE_LICENSE("GPL");

/* enum tvp515x_std - enum for supported standards */
enum tvp515x_std {
	STD_NTSC_MJ = 0,
	STD_PAL_BDGHIN,
	STD_INVALID
};

/**
 * struct tvp515x_std_info - Structure to store standard informations
 * @width: Line width in pixels
 * @height:Number of active lines
 * @video_std: Value to write in REG_VIDEO_STD register
 * @standard: v4l2 standard structure information
 */
struct tvp515x_std_info {
	unsigned long width;
	unsigned long height;
	u8 video_std;
	struct v4l2_standard standard;
	unsigned int mbus_code;
	struct v4l2_mbus_framefmt format;
};

static struct tvp515x_reg tvp515x_reg_list_default[0x40];

static int tvp515x_s_stream(struct v4l2_subdev *sd, int enable);
/**
 * struct tvp515x_decoder - TVP5150/51 decoder object
 * @sd: Subdevice Slave handle
 * @tvp515x_regs: copy of hw's regs with preset values.
 * @pdata: Board specific
 * @ver: Chip version
 * @streaming: TVP5150/51 decoder streaming - enabled or disabled.
 * @current_std: Current standard
 * @num_stds: Number of standards
 * @std_list: Standards list
 * @input: Input routing at chip level
 * @output: Output routing at chip level
 */
struct tvp515x_decoder {
	struct v4l2_subdev sd;
	struct tvp515x_reg tvp515x_regs[ARRAY_SIZE(tvp515x_reg_list_default)];
	const struct tvp515x_platform_data *pdata;
	struct media_pad pad;

	int ver;
	int streaming;

	enum tvp515x_std current_std;
	int num_stds;
	const struct tvp515x_std_info *std_list;
	/* Input and Output Routing parameters */
	u32 input;
	u32 output;
};

/* tvp515x default register values */
static struct tvp515x_reg tvp515x_reg_list_default[] = {
	{ TOK_WRITE,TVP5150_VD_IN_SRC_SEL_1, 0x02 },/* 0x00 */
	{ TOK_WRITE, TVP5150_ANAL_CHL_CTL, 0x15 },/* 0x01 */
	{ TOK_WRITE, TVP5150_OP_MODE_CTL, 0x00 },/* 0x02 */
	{ TOK_WRITE, TVP5150_MISC_CTL, 0x6D },/* 0x03 Enable YUV OUTPUTS*/
	{ TOK_SKIP, TVP5150_AUTOSW_MSK, 0xDC },/* 0x04*/
	{ TOK_SKIP, TVP5150_RES_05, 0x0 },/* 0x05*/
	{ TOK_WRITE, TVP5150_COLOR_KIL_THSH_CTL, 0x10 },/* 0x06 */
	{ TOK_WRITE, TVP5150_LUMA_PROC_CTL_1, 0x60 },/* 0x07 */
	{ TOK_WRITE, TVP5150_LUMA_PROC_CTL_2, 0x00 },/* 0x08 */
	{ TOK_WRITE, TVP5150_BRIGHT_CTL, 0x70 },/* 0x09 */
	{ TOK_WRITE, TVP5150_SATURATION_CTL, 0x80 },/* 0x0a */
	{ TOK_WRITE, TVP5150_HUE_CTL, 0xF7 },/* 0x0b */
	{ TOK_WRITE, TVP5150_CONTRAST_CTL, 0x90 },/* 0x0c */
	{ TOK_WRITE, TVP5150_DATA_RATE_SEL, 0x47 },/* 0x0d */
	{ TOK_WRITE, TVP5150_LUMA_PROC_CTL_3, 0x00 },/* 0x0e */
	{ TOK_WRITE, TVP5150_CONF_SHARED_PIN, 0x08 },/* 0x0f */
	{ TOK_TERM, 0, 0 },
};

/**
 * Supported standards -
 *
 * Currently supports two standards only, need to add support for rest of the
 * modes, like SECAM, etc...
 */
static const struct tvp515x_std_info tvp515x_std_list[] = {
	/* Standard: STD_NTSC_MJ */
	[STD_NTSC_MJ] = {
		.width = NTSC_NUM_ACTIVE_PIXELS,
		.height = NTSC_NUM_ACTIVE_LINES,
		.video_std = VIDEO_STD_NTSC_MJ_BIT,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_2X8,
		.standard = {
			.index = 0,
			.id = V4L2_STD_NTSC,
			.name = "NTSC",
			.frameperiod = {1001, 30000},
			.framelines = 525
		},
		.format = {
			.width = NTSC_NUM_ACTIVE_PIXELS,
			.height = NTSC_NUM_ACTIVE_LINES,
			.code = V4L2_MBUS_FMT_UYVY8_2X8,
			.field = V4L2_FIELD_INTERLACED,
			.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},
	},
	/* Standard: STD_PAL_BDGHIN */
	[STD_PAL_BDGHIN] = {
		.width = PAL_NUM_ACTIVE_PIXELS,
		.height = PAL_NUM_ACTIVE_LINES,
		.video_std = VIDEO_STD_PAL_BDGHIN_BIT,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_2X8,
		.standard = {
			.index = 1,
			.id = V4L2_STD_PAL,
			.name = "PAL",
			.frameperiod = {1, 25},
			.framelines = 625
		},
		.format = {
			.width = PAL_NUM_ACTIVE_PIXELS,
			.height = PAL_NUM_ACTIVE_LINES,
			.code = V4L2_MBUS_FMT_UYVY8_2X8,
			.field = V4L2_FIELD_INTERLACED,
			.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},
	},
	/* Standard: need to add for additional standard */
};


static inline struct tvp515x_decoder *to_decoder(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tvp515x_decoder, sd);
}


/**
 * tvp515x_read_reg() - Read a value from a register in an TVP5150/51.
 * @sd: ptr to v4l2_subdev struct
 * @reg: TVP5150/51 register address
 *
 * Returns value read if successful, or non-zero (-1) otherwise.
 */
static int tvp515x_read_reg(struct v4l2_subdev *sd, u8 reg)
{
	int err, retry = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

read_again:

	err = i2c_smbus_read_byte_data(client, reg);
	if (err < 0) {
		if (retry <= I2C_RETRY_COUNT) {
			v4l2_warn(sd, "Read: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto read_again;
		}
	}

	return err;
}

/**
 * tvp515x_write_reg() - Write a value to a register in TVP5150/51
 * @sd: ptr to v4l2_subdev struct
 * @reg: TVP5150/51 register address
 * @val: value to be written to the register
 *
 * Write a value to a register in an TVP5150/51 decoder device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp515x_write_reg(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int err, retry = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

write_again:

	err = i2c_smbus_write_byte_data(client, reg, val);
	if (err) {
		if (retry <= I2C_RETRY_COUNT) {
			v4l2_warn(sd, "Write: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto write_again;
		}
	}

	return err;
}

/**
 * tvp515x_write_regs() : Initializes a list of TVP5150/51 registers
 * @sd: ptr to v4l2_subdev struct
 * @reglist: list of TVP5150/51 registers and values
 *
 * Initializes a list of TVP5150/51 registers:-
 *		if token is TOK_TERM, then entire write operation terminates
 *		if token is TOK_DELAY, then a delay of 'val' msec is introduced
 *		if token is TOK_SKIP, then the register write is skipped
 *		if token is TOK_WRITE, then the register write is performed
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp515x_write_regs(struct v4l2_subdev *sd,
			      const struct tvp515x_reg reglist[])
{
	int err;
	const struct tvp515x_reg *next = reglist;

	for (; next->token != TOK_TERM; next++) {
		if (next->token == TOK_DELAY) {
			msleep(next->val);
			continue;
		}

		if (next->token == TOK_SKIP)
			continue;

		err = tvp515x_write_reg(sd, next->reg, (u8) next->val);
		if (err) {
			v4l2_err(sd, "Write failed. Err[%d]\n", err);
			return err;
		}
	}
	return 0;
}

/**
 * tvp515x_query_current_std() : Query the current standard detected by TVP5150/51
 * @sd: ptr to v4l2_subdev struct
 *
 * Returns the current standard detected by TVP5150/51, STD_INVALID if there is no
 * standard detected.
 */
static enum tvp515x_std tvp515x_query_current_std(struct v4l2_subdev *sd)
{
	u8 std, std_status;

	std = tvp515x_read_reg(sd, TVP5150_VIDEO_STD);
	if ((std & VIDEO_STD_MASK) == VIDEO_STD_AUTO_SWITCH_BIT)
		/* use the standard status register */
		std_status = tvp515x_read_reg(sd, TVP5150_STATUS_REG_5);
	else
		/* use the standard register itself */
		std_status = std;

	switch (std_status & VIDEO_STD_MASK) {
	case VIDEO_STD_NTSC_MJ_BIT:
		return STD_NTSC_MJ;

	case VIDEO_STD_PAL_BDGHIN_BIT:
		return STD_PAL_BDGHIN;

	default:
		return STD_INVALID;
	}

	return STD_INVALID;
}

/* TVP5150/51 register dump function */
static void tvp515x_reg_dump(struct v4l2_subdev *sd)
{
	v4l2_info(sd,"Video input source selection #1 = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_VD_IN_SRC_SEL_1));
	v4l2_info(sd,"Analog channel controls = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_ANAL_CHL_CTL));
	v4l2_info(sd,"Operation mode controls = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_OP_MODE_CTL));
	v4l2_info(sd,"Miscellaneous controls = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_MISC_CTL));
	v4l2_info(sd,"Autoswitch mask= 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_AUTOSW_MSK));
	v4l2_info(sd,"Color killer threshold control = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_COLOR_KIL_THSH_CTL));
	v4l2_info(sd,"Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
			tvp515x_read_reg(sd, TVP5150_LUMA_PROC_CTL_1),
			tvp515x_read_reg(sd, TVP5150_LUMA_PROC_CTL_2),
			tvp515x_read_reg(sd, TVP5150_LUMA_PROC_CTL_3));
	v4l2_info(sd,"Brightness control = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_BRIGHT_CTL));
	v4l2_info(sd,"Color saturation control = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_SATURATION_CTL));
	v4l2_info(sd,"Hue control = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_HUE_CTL));
	v4l2_info(sd,"Contrast control = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_CONTRAST_CTL));
	v4l2_info(sd,"Outputs and data rates select = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_DATA_RATE_SEL));
	v4l2_info(sd,"Configuration shared pins = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_CONF_SHARED_PIN));
	v4l2_info(sd,"Active video cropping start = 0x%02x%02x\n",
			tvp515x_read_reg(sd, TVP5150_ACT_VD_CROP_ST_MSB),
			tvp515x_read_reg(sd, TVP5150_ACT_VD_CROP_ST_LSB));
	v4l2_info(sd,"Active video cropping stop  = 0x%02x%02x\n",
			tvp515x_read_reg(sd, TVP5150_ACT_VD_CROP_STP_MSB),
			tvp515x_read_reg(sd, TVP5150_ACT_VD_CROP_STP_LSB));
	v4l2_info(sd,"Genlock/RTC = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_GENLOCK));
	v4l2_info(sd,"Horizontal sync start = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_HORIZ_SYNC_START));
	v4l2_info(sd,"Vertical blanking start = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_VERT_BLANKING_START));
	v4l2_info(sd,"Vertical blanking stop = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_VERT_BLANKING_STOP));
	v4l2_info(sd,"Chrominance processing control #1 and #2 = %02x %02x\n",
			tvp515x_read_reg(sd, TVP5150_CHROMA_PROC_CTL_1),
			tvp515x_read_reg(sd, TVP5150_CHROMA_PROC_CTL_2));
	v4l2_info(sd,"Interrupt reset register B = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_INT_RESET_REG_B));
	v4l2_info(sd,"Interrupt enable register B = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_INT_ENABLE_REG_B));
	v4l2_info(sd,"Interrupt configuration register B = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_INTT_CONFIG_REG_B));
	v4l2_info(sd,"Video standard = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_VIDEO_STD));
	v4l2_info(sd,"Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_CB_GAIN_FACT),
			tvp515x_read_reg(sd, TVP5150_CR_GAIN_FACTOR));
	v4l2_info(sd,"Macrovision on counter = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_MACROVISION_ON_CTR));
	v4l2_info(sd,"Macrovision off counter = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_MACROVISION_OFF_CTR));
	v4l2_info(sd,"ITU-R BT.656.%d timing(TVP5150AM1 only)\n",
			(tvp515x_read_reg(sd, TVP5150_REV_SELECT) & 1) ?
					3 : 4);
	v4l2_info(sd,"Device ID = %02x%02x\n",
			tvp515x_read_reg(sd, TVP5150_MSB_DEV_ID),
			tvp515x_read_reg(sd, TVP5150_LSB_DEV_ID));
	v4l2_info(sd,"ROM version = (hex) %02x.%02x\n",
			tvp515x_read_reg(sd, TVP5150_ROM_MAJOR_VER),
			tvp515x_read_reg(sd, TVP5150_ROM_MINOR_VER));
	v4l2_info(sd,"Vertical line count = 0x%02x%02x\n",
			tvp515x_read_reg(sd, TVP5150_VERT_LN_COUNT_MSB),
			tvp515x_read_reg(sd, TVP5150_VERT_LN_COUNT_LSB));
	v4l2_info(sd,"Interrupt status register B = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_INT_STATUS_REG_B));
	v4l2_info(sd,"Interrupt active register B = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_INT_ACTIVE_REG_B));
	v4l2_info(sd,"Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",
			tvp515x_read_reg(sd, TVP5150_STATUS_REG_1),
			tvp515x_read_reg(sd, TVP5150_STATUS_REG_2),
			tvp515x_read_reg(sd, TVP5150_STATUS_REG_3),
			tvp515x_read_reg(sd, TVP5150_STATUS_REG_4),
			tvp515x_read_reg(sd, TVP5150_STATUS_REG_5));

	v4l2_info(sd,"Teletext filter enable = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_TELETEXT_FIL_ENA));
	v4l2_info(sd,"Interrupt status register A = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_INT_STATUS_REG_A));
	v4l2_info(sd,"Interrupt enable register A = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_INT_ENABLE_REG_A));
	v4l2_info(sd,"Interrupt configuration = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_INT_CONF));
	v4l2_info(sd,"VDP status register = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_VDP_STATUS_REG));
	v4l2_info(sd,"FIFO word count = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_FIFO_WORD_COUNT));
	v4l2_info(sd,"FIFO interrupt threshold = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_FIFO_INT_THRESHOLD));
	v4l2_info(sd,"FIFO reset = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_FIFO_RESET));
	v4l2_info(sd,"Line number interrupt = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_LINE_NUMBER_INT));
	v4l2_info(sd,"Pixel alignment register = 0x%02x%02x\n",
			tvp515x_read_reg(sd, TVP5150_PIX_ALIGN_REG_HIGH),
			tvp515x_read_reg(sd, TVP5150_PIX_ALIGN_REG_LOW));
	v4l2_info(sd,"FIFO output control = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_FIFO_OUT_CTRL));
	v4l2_info(sd,"Full field enable = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_FULL_FIELD_ENA));
	v4l2_info(sd,"Full field mode register = 0x%02x\n",
			tvp515x_read_reg(sd, TVP5150_FULL_FIELD_MODE_REG));

}

/**
 * tvp515x_configure() - Configure the TVP5150/51 registers
 * @sd: ptr to v4l2_subdev struct
 * @decoder: ptr to tvp515x_decoder structure
 *
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp515x_configure(struct v4l2_subdev *sd,
		struct tvp515x_decoder *decoder)
{
	int err;

	/* common register initialization */
	err =
	    tvp515x_write_regs(sd, decoder->tvp515x_regs);
	if (err)
		return err;

	if (debug)
		tvp515x_reg_dump(sd);

	return 0;
}

/**
 * tvp515x_detect() - Detect if an tvp515x is present, and if so which revision.
 * @sd: pointer to standard V4L2 sub-device structure
 * @decoder: pointer to tvp515x_decoder structure
 *
 * A device is considered to be detected if the chip ID (LSB and MSB)
 * registers match the expected values.
 * Any value of the rom version register is accepted.
 * Returns ENODEV error number if no device is detected, or zero
 * if a device is detected.
 */
static int tvp515x_detect(struct v4l2_subdev *sd,
		struct tvp515x_decoder *decoder)
{
	u8 chip_id_msb, chip_id_lsb, rom_ver_maj,rom_ver_min;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 chip_id;
	chip_id_msb = tvp515x_read_reg(sd, TVP5150_MSB_DEV_ID);
	chip_id_lsb = tvp515x_read_reg(sd, TVP5150_LSB_DEV_ID);
	rom_ver_maj = tvp515x_read_reg(sd, TVP5150_ROM_MAJOR_VER);
	rom_ver_min = tvp515x_read_reg(sd, TVP5150_ROM_MINOR_VER);
	chip_id = (chip_id_msb <<8) | chip_id_lsb;
	v4l2_dbg(1, debug, sd,
		 "chip id detected msb:0x%x rom version:%d.%d\n",
		 chip_id, rom_ver_maj,rom_ver_min);

	switch(chip_id){
	case 0x5151:
	case 0x5150:
		break;
	default:
		v4l2_err(sd, "chip id mismatch msb:0x%x lsb:0x%x\n",
				chip_id_msb, chip_id_lsb);
		return -ENODEV;
	}

	decoder->ver = (rom_ver_maj << 8) | rom_ver_min;

	v4l2_info(sd, "%s (Version - 0x%.2x) found at 0x%x (%s)\n",
			client->name, decoder->ver,
			client->addr << 1, client->adapter->name);
	return 0;
}

/**
 * tvp515x_querystd() - V4L2 decoder interface handler for querystd
 * @sd: pointer to standard V4L2 sub-device structure
 * @std_id: standard V4L2 std_id ioctl enum
 *
 * Returns the current standard detected by TVP5150/51. If no active input is
 * detected then *std_id is set to 0 and the function returns 0.
 */
static int tvp515x_querystd(struct v4l2_subdev *sd, v4l2_std_id *std_id)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);
	enum tvp515x_std current_std;
	enum tvp515x_input input_sel;
	u8 sync_lock_status, lock_mask;

	if (std_id == NULL)
		return -EINVAL;

	*std_id = V4L2_STD_UNKNOWN;

	/* query the current standard */
	current_std = tvp515x_query_current_std(sd);
	if (current_std == STD_INVALID)
		return 0;

	input_sel = decoder->input;

#ifdef CONFIG_VIDEO_TVP515X_SOM336_INPUT_SELECTOR
	input_sel = (input_sel%2)<<1;
#endif

	switch (input_sel) {
	case INPUT_CVBS_P1A:
	case INPUT_CVBS_P1B:
		lock_mask = STATUS_CLR_SUBCAR_LOCK_BIT |
			STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;

	case INPUT_SVIDEO_PI1A_P1B:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;
		/*Need to add other interfaces*/
	default:
		return -EINVAL;
	}
	/* check whether signal is locked */
	sync_lock_status = tvp515x_read_reg(sd, TVP5150_STATUS_REG_1);
	if (lock_mask != (sync_lock_status & lock_mask))
		return 0;	/* No input detected */

	*std_id = decoder->std_list[current_std].standard.id;

	v4l2_dbg(1, debug, sd, "Current STD: %s\n",
			decoder->std_list[current_std].standard.name);
	return 0;
}

/**
 * tvp515x_s_std() - V4L2 decoder interface handler for s_std
 * @sd: pointer to standard V4L2 sub-device structure
 * @std_id: standard V4L2 v4l2_std_id ioctl enum
 *
 * If std_id is supported, sets the requested standard. Otherwise, returns
 * -EINVAL
 */
static int tvp515x_s_std(struct v4l2_subdev *sd, v4l2_std_id std_id)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);
	int err, i;

	for (i = 0; i < decoder->num_stds; i++)
		if (std_id & decoder->std_list[i].standard.id)
			break;

	if ((i == decoder->num_stds) || (i == STD_INVALID))
		return -EINVAL;

	err = tvp515x_write_reg(sd, TVP5150_VIDEO_STD,
				decoder->std_list[i].video_std);
	if (err)
		return err;

	decoder->current_std = i;
	decoder->tvp515x_regs[TVP5150_VIDEO_STD].val =
		decoder->std_list[i].video_std;

	v4l2_dbg(1, debug, sd, "Standard set to: %s\n",
			decoder->std_list[i].standard.name);
	return 0;
}

/**
 * tvp515x_s_routing() - V4L2 decoder interface handler for s_routing
 * @sd: pointer to standard V4L2 sub-device structure
 * @input: input selector for routing the signal
 * @output: output selector for routing the signal
 * @config: config value. Not used
 *
 * If index is valid, selects the requested input. Otherwise, returns -EINVAL if
 * the input is not supported or there is no active signal present in the
 * selected input.
 */
static int tvp515x_s_routing(struct v4l2_subdev *sd,
				u32 input, u32 output, u32 config)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);
	int err;
	enum tvp515x_input input_sel;
	enum tvp515x_output output_sel;
	u8 sync_lock_status, lock_mask;
	int try_count = LOCK_RETRY_COUNT;

	if ((input >= INPUT_INVALID) ||
			(output >= OUTPUT_INVALID))
		/* Index out of bound */
		return -EINVAL;

	/*
	 * For the sequence streamon -> streamoff and again s_input
	 * it fails to lock the signal, since streamoff puts tvp515x
	 * into power off state which leads to failure in sub-sequent s_input.
	 *
	 * So power up the tvp515x device here, since it is important to lock
	 * the signal at this stage.
	 */
	if (!decoder->streaming)
		tvp515x_s_stream(sd, 1);

	input_sel = input;
	output_sel = output;

#ifdef CONFIG_VIDEO_TVP515X_SOM336_INPUT_SELECTOR
	input_sel = (input_sel%2)<<1;
	gpio_set_value(OMAP3_EGF_TVP5150_INPUT_SEL_GPIO,input/2);
#endif
	err = tvp515x_write_reg(sd, TVP5150_VD_IN_SRC_SEL_1, input_sel);
	if (err)
		return err;

	output_sel |= tvp515x_read_reg(sd,
			TVP5150_DATA_RATE_SEL) & 0x7;
	err = tvp515x_write_reg(sd, TVP5150_DATA_RATE_SEL,
			output_sel);
	if (err)
		return err;

	decoder->tvp515x_regs[TVP5150_VD_IN_SRC_SEL_1].val = input_sel;
	decoder->tvp515x_regs[TVP5150_DATA_RATE_SEL].val = output_sel;


	switch (input_sel) {
	case INPUT_CVBS_P1A:
	case INPUT_CVBS_P1B:
		lock_mask = STATUS_CLR_SUBCAR_LOCK_BIT | STATUS_HORZ_SYNC_LOCK_BIT
				| STATUS_VIRT_SYNC_LOCK_BIT;
		break;

	case INPUT_SVIDEO_PI1A_P1B:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT | STATUS_VIRT_SYNC_LOCK_BIT;
		break;
		/*Need to add other interfaces*/
	default:
		v4l2_dbg(1, debug, sd, "Input_sel: %d Invalid\n", input_sel);
		return -EINVAL;
	}

	while (try_count-- > 0) {
		/* Allow decoder to sync up with new input */
		msleep(LOCK_RETRY_DELAY);

		sync_lock_status = tvp515x_read_reg(sd, TVP5150_STATUS_REG_1);
		if (lock_mask == (sync_lock_status & lock_mask))
			/* Input detected */
			break;
	}

	if (try_count < 0)
		return -EINVAL;

	decoder->input = input;
	decoder->output = output;

	v4l2_dbg(1, debug, sd, "Input set to: %d\n", input);

	return 0;
}

/**
 * tvp515x_queryctrl() - V4L2 decoder interface handler for queryctrl
 * @sd: pointer to standard V4L2 sub-device structure
 * @qctrl: standard V4L2 v4l2_queryctrl structure
 *
 * If the requested control is supported, returns the control information.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int
tvp515x_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qctrl)
{
	int err = -EINVAL;

	if (qctrl == NULL)
		return err;

	switch (qctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		/* Brightness supported is (0-255), */
		err = v4l2_ctrl_query_fill(qctrl, 0, 255, 1, 128);
		break;
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
		/**
		 * Saturation and Contrast supported is -
		 *	Contrast: 0 - 255 (Default - 128)
		 *	Saturation: 0 - 255 (Default - 128)
		 */
		err = v4l2_ctrl_query_fill(qctrl, 0, 255, 1, 128);
		break;
	case V4L2_CID_HUE:
		/* Hue Supported is -
		 *	Hue - -180 - +180 (Default - 0, Step - +180)
		 */
		err = v4l2_ctrl_query_fill(qctrl, -180, 180, 180, 0);
		break;
	case V4L2_CID_AUTOGAIN:
		/**
		 * Auto Gain supported is -
		 * 	0 - 1 (Default - 1)
		 */
		err = v4l2_ctrl_query_fill(qctrl, 0, 1, 1, 1);
		break;
	default:
		v4l2_err(sd, "invalid control id %d\n", qctrl->id);
		return err;
	}

	v4l2_dbg(1, debug, sd, "Query Control:%s: Min - %d, Max - %d, Def - %d\n",
			qctrl->name, qctrl->minimum, qctrl->maximum,
			qctrl->default_value);

	return err;
}

/**
 * tvp515x_g_ctrl() - V4L2 decoder interface handler for g_ctrl
 * @sd: pointer to standard V4L2 sub-device structure
 * @ctrl: pointer to v4l2_control structure
 *
 * If the requested control is supported, returns the control's current
 * value from the decoder. Otherwise, returns -EINVAL if the control is not
 * supported.
 */
static int
tvp515x_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);

	if (ctrl == NULL)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = decoder->tvp515x_regs[TVP5150_BRIGHT_CTL].val;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = decoder->tvp515x_regs[TVP5150_CONTRAST_CTL].val;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = decoder->tvp515x_regs[TVP5150_SATURATION_CTL].val;
		break;
	case V4L2_CID_HUE:
		ctrl->value = decoder->tvp515x_regs[TVP5150_HUE_CTL].val;
		if (ctrl->value == 0x7F)
			ctrl->value = 180;
		else if (ctrl->value == 0x80)
			ctrl->value = -180;
		else
			ctrl->value = 0;

		break;
	default:
		v4l2_err(sd, "invalid control id %d\n", ctrl->id);
		return -EINVAL;
	}

	v4l2_dbg(1, debug, sd, "Get Control: ID - %d - %d\n",
			ctrl->id, ctrl->value);
	return 0;
}

/**
 * tvp515x_s_ctrl() - V4L2 decoder interface handler for s_ctrl
 * @sd: pointer to standard V4L2 sub-device structure
 * @ctrl: pointer to v4l2_control structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW. Otherwise, returns -EINVAL if the control is not supported.
 */
static int
tvp515x_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);
	int err = -EINVAL, value;

	if (ctrl == NULL)
		return err;

	value = ctrl->value;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l2_err(sd, "invalid brightness setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = tvp515x_write_reg(sd, TVP5150_BRIGHT_CTL,
				value);
		if (err)
			return err;

		decoder->tvp515x_regs[TVP5150_BRIGHT_CTL].val = value;
		break;
	case V4L2_CID_CONTRAST:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l2_err(sd, "invalid contrast setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = tvp515x_write_reg(sd, TVP5150_CONTRAST_CTL, value);
		if (err)
			return err;

		decoder->tvp515x_regs[TVP5150_CONTRAST_CTL].val = value;
		break;
	case V4L2_CID_SATURATION:
		if (ctrl->value < 0 || ctrl->value > 255) {
			v4l2_err(sd, "invalid saturation setting %d\n",
					ctrl->value);
			return -ERANGE;
		}
		err = tvp515x_write_reg(sd, TVP5150_SATURATION_CTL, value);
		if (err)
			return err;

		decoder->tvp515x_regs[TVP5150_SATURATION_CTL].val = value;
		break;
	case V4L2_CID_HUE:
		if (value == 180)
			value = 0x7F;
		else if (value == -180)
			value = 0x80;
		else if (value == 0)
			value = 0;
		else {
			v4l2_err(sd, "invalid hue setting %d\n", ctrl->value);
			return -ERANGE;
		}
		err = tvp515x_write_reg(sd, TVP5150_HUE_CTL, value);
		if (err)
			return err;

		decoder->tvp515x_regs[TVP5150_HUE_CTL].val = value;
		break;
	default:
		v4l2_err(sd, "invalid control id %d\n", ctrl->id);
		return err;
	}

	v4l2_dbg(1, debug, sd, "Set Control: ID - %d - %d\n",
			ctrl->id, ctrl->value);

	return err;
}

/**
 * tvp515x_enum_mbus_fmt() - V4L2 decoder interface handler for enum_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @index: index of pixelcode to retrieve
 * @code: receives the pixelcode
 *
 * Enumerates supported mediabus formats
 */
static int
tvp515x_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned index,
					enum v4l2_mbus_pixelcode *code)
{
	if (index)
		return -EINVAL;

	*code = V4L2_MBUS_FMT_UYVY8_2X8;
	return 0;
}

/**
 * tvp515x_mbus_fmt_cap() - V4L2 decoder interface handler for try/s/g_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @f: pointer to the mediabus format structure
 *
 * Negotiates the image capture size and mediabus format.
 */
static int
tvp515x_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *f)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);
	enum tvp515x_std current_std;

	if (f == NULL)
		return -EINVAL;

	/* Calculate height and width based on current standard */
	current_std = decoder->current_std;

	f->code = V4L2_MBUS_FMT_UYVY8_2X8;
	f->width = decoder->std_list[current_std].width;
	f->height = decoder->std_list[current_std].height;
	f->field = V4L2_FIELD_INTERLACED;
	f->colorspace = V4L2_COLORSPACE_SMPTE170M;

	v4l2_dbg(1, debug, sd, "MBUS_FMT: Width - %d, Height - %d\n",
			f->width, f->height);
	return 0;
}

/**
 * tvp515x_g_parm() - V4L2 decoder interface handler for g_parm
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the decoder's video CAPTURE parameters.
 */
static int
tvp515x_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);
	struct v4l2_captureparm *cparm;
	enum tvp515x_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;

	/* get the current standard */
	current_std = decoder->current_std;

	cparm = &a->parm.capture;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe =
		decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/**
 * tvp515x_s_parm() - V4L2 decoder interface handler for s_parm
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the decoder to use the input parameters, if possible. If
 * not possible, returns the appropriate error code.
 */
static int
tvp515x_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);
	struct v4l2_fract *timeperframe;
	enum tvp515x_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;

	timeperframe = &a->parm.capture.timeperframe;

	/* get the current standard */
	current_std = decoder->current_std;

	*timeperframe =
	    decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/**
 * tvp515x_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable, if possible.
 */
static int tvp515x_s_stream(struct v4l2_subdev *sd, int enable)
{
	int err = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tvp515x_decoder *decoder = to_decoder(sd);

	if (decoder->streaming == enable)
		return 0;

	switch (enable) {
	case 0:
	{
		v4l2_dbg(1, debug, sd, "Stop Streaming\n");
		/* Power Down Sequence */
		err = tvp515x_write_reg(sd, TVP5150_OP_MODE_CTL, 0x01);
		if (err) {
			v4l2_err(sd, "Unable to turn off decoder\n");
			return err;
		}
		decoder->streaming = enable;
		break;
	}
	case 1:
	{
		struct tvp515x_reg *int_seq = (struct tvp515x_reg *)
				client->driver->id_table->driver_data;

		v4l2_dbg(1, debug, sd, "Start Streaming\n");
		/* Power Up Sequence */
		err = tvp515x_write_regs(sd, int_seq);
		if (err) {
			v4l2_err(sd, "Unable to turn on decoder\n");
			return err;
		}
		/* Detect if not already detected */
		err = tvp515x_detect(sd, decoder);
		if (err) {
			v4l2_err(sd, "Unable to detect decoder\n");
			return err;
		}
		err = tvp515x_configure(sd, decoder);
		if (err) {
			v4l2_err(sd, "Unable to configure decoder\n");
			return err;
		}
		decoder->streaming = enable;
		break;
	}
	default:
		err = -ENODEV;
		break;
	}

	v4l2_dbg(1, debug, sd, "TVP5150_OP_MODE_CTL=%0x\n",tvp515x_read_reg(sd, TVP5150_OP_MODE_CTL));

	return err;
}

static int tvp515x_s_power(struct v4l2_subdev *sd, int on)
{
	struct tvp515x_decoder *decoder = to_decoder(sd);

	if (decoder->pdata && decoder->pdata->s_power)
		return decoder->pdata->s_power(sd, on);

	return 0;
}

/*
 * tvp515x_enum_mbus_code - V4L2 sensor interface handler for pad_ops
 * @subdev: pointer to standard V4L2 sub-device structure
 * @fh: pointer to standard V4L2 sub-device file handle
 * @code: pointer to v4l2_subdev_pad_mbus_code_enum structure
 *
 */
static int tvp515x_enum_mbus_code(struct v4l2_subdev *subdev,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(tvp515x_std_list))
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_UYVY8_2X8;

	return 0;
}

static int tvp515x_get_pad_format(struct v4l2_subdev *subdev,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_format *fmt)
{
	struct tvp515x_decoder *decoder = to_decoder(subdev);
	enum tvp515x_std current_std;

	/* query the current standard */
	current_std = tvp515x_query_current_std(subdev);
	if (current_std == STD_INVALID) {
		v4l2_err(subdev, "Unable to query std\n");
		return 0;
	}

	fmt->format = decoder->std_list[current_std].format;

	return 0;
}

static int tvp515x_set_pad_format(struct v4l2_subdev *subdev,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_format *fmt)
{
	struct tvp515x_decoder *decoder = to_decoder(subdev);
	enum tvp515x_std current_std;

	/* query the current standard */
	current_std = tvp515x_query_current_std(subdev);
	if (current_std == STD_INVALID) {
		v4l2_err(subdev, "Unable to query std\n");
		return 0;
	}

	fmt->format.width = decoder->std_list[current_std].width;
	fmt->format.height = decoder->std_list[current_std].height;
	fmt->format.code = V4L2_MBUS_FMT_UYVY8_2X8;
	fmt->format.field = V4L2_FIELD_INTERLACED;
	fmt->format.colorspace = V4L2_COLORSPACE_SMPTE170M;

	return 0;
}

static int tvp515x_enum_frame_size(struct v4l2_subdev *subdev,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_frame_size_enum *fse)
{
	struct tvp515x_decoder *decoder = to_decoder(subdev);
	enum tvp515x_std current_std;

	if (fse->code != V4L2_MBUS_FMT_UYVY8_2X8)
		return -EINVAL;

	/* query the current standard */
	current_std = tvp515x_query_current_std(subdev);
	if (current_std == STD_INVALID) {
		v4l2_err(subdev, "Unable to query std\n");
		return 0;
	}

	fse->min_width = decoder->std_list[current_std].width;
	fse->min_height = decoder->std_list[current_std].height;
	fse->max_width = fse->min_width;
	fse->max_height = fse->min_height;

	return 0;
}

static int tvp515x_registered(struct v4l2_subdev *subdev)
{
	enum tvp515x_std current_std;

	tvp515x_s_stream(subdev, 1);
	/* query the current standard */
	current_std = tvp515x_query_current_std(subdev);
	if (current_std == STD_INVALID) {
		v4l2_err(subdev, "Unable to query std\n");
		return 0;
	}

	tvp515x_s_stream(subdev, 0);

	return 0;
}

/* --------------------------------------------------------------------------
 * V4L2 subdev file operations
 */
static int tvp515x_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	enum tvp515x_std current_std;

	tvp515x_s_stream(subdev, 1);
	/* query the current standard */
	current_std = tvp515x_query_current_std(subdev);
	if (current_std == STD_INVALID) {
		v4l2_err(subdev, "Unable to query std\n");
		return 0;
	}

	tvp515x_s_stream(subdev, 0);

	return 0;
}

/* --------------------------------------------------------------------------
 * V4L2 subdev core operations
 */
static int tvp515x_g_chip_ident(struct v4l2_subdev *subdev,
		                struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(subdev);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_TVP5151, 0);
}

static const struct v4l2_subdev_core_ops tvp515x_core_ops = {
	.g_chip_ident = tvp515x_g_chip_ident,
	.queryctrl = tvp515x_queryctrl,
	.g_ctrl = tvp515x_g_ctrl,
	.s_ctrl = tvp515x_s_ctrl,
	.s_std = tvp515x_s_std,
	.s_power = tvp515x_s_power,
};

static const struct v4l2_subdev_internal_ops tvp515x_internal_ops = {
	.registered = tvp515x_registered,
	.open = tvp515x_open,
 };

static const struct v4l2_subdev_video_ops tvp515x_video_ops = {
	.s_routing = tvp515x_s_routing,
	.querystd = tvp515x_querystd,
	.enum_mbus_fmt = tvp515x_enum_mbus_fmt,
	.g_mbus_fmt = tvp515x_mbus_fmt,
	.try_mbus_fmt = tvp515x_mbus_fmt,
	.s_mbus_fmt = tvp515x_mbus_fmt,
	.g_parm = tvp515x_g_parm,
	.s_parm = tvp515x_s_parm,
	.s_stream = tvp515x_s_stream,
};

static const struct v4l2_subdev_pad_ops tvp515x_pad_ops = {
	.enum_mbus_code = tvp515x_enum_mbus_code,
	.enum_frame_size = tvp515x_enum_frame_size,
	.get_fmt = tvp515x_get_pad_format,
	.set_fmt = tvp515x_set_pad_format,
};

static const struct v4l2_subdev_ops tvp515x_ops = {
	.core = &tvp515x_core_ops,
	.video = &tvp515x_video_ops,
	.pad = &tvp515x_pad_ops,
};

static struct tvp515x_decoder tvp515x_dev = {
	.streaming = 0,
	.current_std = STD_NTSC_MJ,
	.std_list = tvp515x_std_list,
	.num_stds = ARRAY_SIZE(tvp515x_std_list),

};

/**
 * tvp515x_probe() - decoder driver i2c probe handler
 * @client: i2c driver client device structure
 * @id: i2c driver id table
 *
 * Register decoder as an i2c client device and V4L2
 * device.
 */
static int
tvp515x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tvp515x_decoder *decoder;
	struct v4l2_subdev *sd;
	int ret;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	if (!client->dev.platform_data) {
		v4l2_err(client, "No platform data!!\n");
		return -ENODEV;
	}

	decoder = kzalloc(sizeof(*decoder), GFP_KERNEL);
	if (!decoder)
		return -ENOMEM;

	/* Initialize the tvp515x_decoder with default configuration */
	*decoder = tvp515x_dev;
	/* Copy default register configuration */
	memcpy(decoder->tvp515x_regs, tvp515x_reg_list_default,
			sizeof(tvp515x_reg_list_default));

	/* Copy board specific information here */
	decoder->pdata = client->dev.platform_data;

	/**
	 * Fetch platform specific data, and configure the
	 * tvp515x_reg_list[] accordingly. Since this is one
	 * time configuration, no need to preserve.
	 */
	/* Set default standard to auto */
	decoder->tvp515x_regs[TVP5150_VIDEO_STD].val =
		VIDEO_STD_AUTO_SWITCH_BIT;

	/* Register with V4L2 layer as slave device */
	sd = &decoder->sd;

	v4l2_i2c_subdev_init(sd, client, &tvp515x_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &tvp515x_internal_ops;
	decoder->pad.flags = MEDIA_PAD_FL_SOURCE;
	decoder->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(&decoder->sd.entity, 1, &decoder->pad, 0);
	if (ret < 0) {
		v4l2_err(client, "failed to register as a media entity!!\n");
		kfree(decoder);
		return ret;
	}

#ifdef CONFIG_VIDEO_TVP515X_SOM336_INPUT_SELECTOR
	ret = gpio_request(OMAP3_EGF_TVP5150_INPUT_SEL_GPIO, "tvp5150 sel input");
	if (ret) {
		printk(KERN_ERR "failed to get tvp5150 sel input\n");
		return ret;
	}
	gpio_direction_output(OMAP3_EGF_TVP5150_INPUT_SEL_GPIO, 0);

#endif
	v4l2_info(sd, "%s decoder driver registered !!\n", sd->name);
	return 0;

}

/**
 * tvp515x_remove() - decoder driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister decoder as an i2c client device and V4L2
 * device. Complement of tvp515x_probe().
 */
static int tvp515x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tvp515x_decoder *decoder = to_decoder(sd);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&decoder->sd.entity);
	kfree(decoder);
#ifdef CONFIG_VIDEO_TVP515X_SOM336_INPUT_SELECTOR
	gpio_free(OMAP3_EGF_TVP5150_INPUT_SEL_GPIO);
#endif
	return 0;
}
/* TVP515x Init/Power on Sequence */
static const struct tvp515x_reg tvp5150_init_reg_seq[] = {
	{TOK_WRITE, TVP5150_CONF_SHARED_PIN, 0x08},
	{TOK_WRITE, TVP5150_ANAL_CHL_CTL, 0x15},
	{TOK_WRITE, TVP5150_MISC_CTL, 0x6d},
	{TOK_WRITE, TVP5150_AUTOSW_MSK, 0x01},
	{TOK_WRITE, TVP5150_DATA_RATE_SEL, 0x47},
	{TOK_WRITE, TVP5150_CHROMA_PROC_CTL_1, 0x0c},
	{TOK_WRITE, TVP5150_CHROMA_PROC_CTL_2, 0x14},
	{TOK_TERM, 0, 0},
};
static const struct tvp515x_reg tvp5151_init_reg_seq[] = {
	{TOK_WRITE, TVP5150_CONF_SHARED_PIN, 0x08},
	{TOK_WRITE, TVP5150_ANAL_CHL_CTL, 0x15},
	{TOK_WRITE, TVP5150_MISC_CTL, 0x6d},
	{TOK_WRITE, TVP5150_AUTOSW_MSK, 0x01},
	{TOK_WRITE, TVP5150_DATA_RATE_SEL, 0x47},
	{TOK_WRITE, TVP5150_CHROMA_PROC_CTL_1, 0x0c},
	{TOK_WRITE, TVP5150_CHROMA_PROC_CTL_2, 0x14},
	{TOK_TERM, 0, 0},
};

/**
 * I2C Device Table -
 *
 * name - Name of the actual device/chip.
 * driver_data - Driver data
 */
static const struct i2c_device_id tvp515x_id[] = {
	{"tvp5150am1", (unsigned long)tvp5150_init_reg_seq},
	{"tvp5151", (unsigned long)tvp5151_init_reg_seq},
	{},
};

MODULE_DEVICE_TABLE(i2c, tvp515x_id);

static struct i2c_driver tvp515x_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = TVP515x_MODULE_NAME,
	},
	.probe = tvp515x_probe,
	.remove = tvp515x_remove,
	.id_table = tvp515x_id,
};

static int __init tvp515x_init(void)
{
	return i2c_add_driver(&tvp515x_driver);
}

static void __exit tvp515x_exit(void)
{
	i2c_del_driver(&tvp515x_driver);
}

module_init(tvp515x_init);
module_exit(tvp515x_exit);
