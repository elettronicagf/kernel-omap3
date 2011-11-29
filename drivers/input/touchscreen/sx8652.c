/*
 * SX8652 based touchscreen and sensor driver
 *
 *  Copyright (c) 2010 Wayne Roberts
 *
 * Using code from:
 *  Copyright (c) 2005 David Brownell
 *  Copyright (c) 2006 Nokia Corporation
 *  Various changes: Imre Deak <imre.deak@nokia.com>
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
*	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/sx8652.h>


#define	MAX_12BIT	((1<<12)-1)

#define SX8652_TS_PENUP_TIME	100

/* analog channels */
#define CH_X	0
#define CH_Y	1
#define CH_Z1	2
#define CH_Z2	3
#define CH_AUX	4
#define CH_SEQ	7

/* commands */
#define SX8652_CMD_WRITEREG		0x00
#define SX8652_CMD_READCHAN		0x20
#define SX8652_CMD_READREG		0x40
#define SX8652_CMD_SELECT		0x80
#define SX8652_CMD_CONVERT		0x90
#define SX8652_CMD_MANAUTO		0xb0
#define SX8652_CMD_PENDET		0xc0
#define SX8652_CMD_PENTRG		0xe0

/* register addresses */
#define SX8652_REG_CTRL0	0x00
#define SX8652_REG_CTRL1	0x01
#define SX8652_REG_CTRL2	0x02
#define SX8652_REG_CHANMSK	0x04
#define SX8652_REG_STATUS	0x05
#define SX8652_REG_RESET	0x1f

/* for POWDLY or SETDLY: */
#define DLY_0_5US	0x00
#define DLY_1_1US	0x01
#define DLY_2_2US	0x02
#define DLY_4_4US	0x03
#define DLY_9US		0x04
#define DLY_18US	0x05
#define DLY_35uS	0x06
#define DLY_71US	0x07
#define DLY_140US	0x08
#define DLY_280US	0x09
#define DLY_570US	0x0a
#define DLY_1_1MS	0x0b
#define DLY_2_3MS	0x0c
#define DLY_4_5MS	0x0d
#define DLY_9MS		0x0e
#define DLY_18MS	0x0f

// RegCtrl1
#define CONDIRQ 	0x20
#define FILT_NONE	0x00
#define FILT_3SA	0x01
#define FILT_5SA	0x02
#define FILT_7SA	0x03

#define CONV_X		0x80
#define CONV_Y		0x40
#define CONV_Z1		0x20
#define CONV_Z2		0x10
#define CONV_AUX	0x08

#define CHAN_MASK	(CONV_X | CONV_Y | CONV_Z1 | CONV_Z2)

#define RESET_VALUE	0xde

#define NUM_READ_REGS	4	/* count of words to read */


struct sx8652 {
	struct input_dev	*input;
	char			phys[32];
	struct spi_device	*spi;
	spinlock_t		lock;
	struct mutex	mutex;
	unsigned		disabled:1;
	struct timer_list	penup_timer;
	u16			model;
	u16			y_plate_ohms;
	u16			pressure_max;

	u8 pen_down;

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	struct attribute_group	*attr_group;
	struct device		*hwmon;
#endif


	struct spi_message	read_msg;
	struct spi_transfer	read_xfer;
	u8 read_cmd;
	u8 data[(NUM_READ_REGS << 1) + 1];
	u8 spi_active;

	int			(*get_pendown_state)(void);
	void			(*wait_for_sync)(void);
};

static struct attribute *sx8652_attributes[] = {
	NULL,
};

static struct attribute_group sx8652_attr_group = {
	.attrs = sx8652_attributes,
};


static void sx8652_ts_penup_timer_handler(unsigned long data)
{
    struct sx8652 *ts = (struct sx8652 *)data;

	printk(KERN_ERR "penup_timer_handler %d spi_active=%d\n", ts->pen_down, ts->spi_active);
	input_report_abs(ts->input, ABS_PRESSURE, 0);
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_sync(ts->input);

	ts->pen_down = 0;
}


static int __devinit setup_pendown(struct spi_device *spi, struct sx8652 *ts)
{
	struct sx8652_platform_data *pdata = spi->dev.platform_data;
	int err;

	if (!pdata->get_pendown_state && !gpio_is_valid(pdata->gpio_pendown)) {
		dev_err(&spi->dev, "no get_pendown_state nor gpio_pendown?\n");
		return -EINVAL;
	}

	if (pdata->get_pendown_state) {
		ts->get_pendown_state = pdata->get_pendown_state;
		printk(KERN_ERR "get_pendown_state from pdata: %p\n", ts->get_pendown_state);
		return 0;
	}

	err = gpio_request(pdata->gpio_pendown, "sx8652_pendown");
	if (err) {
		dev_err(&spi->dev, "failed to request pendown GPIO%d\n",
				pdata->gpio_pendown);
		return err;
	}

	return 0;
}


static irqreturn_t sx8652_irq(int irq, void *handle)
{
	struct sx8652 *ts = handle;
	int status;

	/* If insufficient pullup resistor on nIRQ line:
	 * may need to make sure that pen is really down here, due to spurious interrupts  */
	if (likely(ts->get_pendown_state())) {
		if (ts->spi_active)
			return IRQ_HANDLED;


		ts->spi_active = 1;
		status = spi_async(ts->spi, &ts->read_msg);
		if (status)
			dev_err(&ts->spi->dev, "spi_async --> %d\n", status);

	    /* kick pen up timer */
		mod_timer(&ts->penup_timer,
			jiffies + msecs_to_jiffies(SX8652_TS_PENUP_TIME));
	}

	return IRQ_HANDLED;
}


static void null_wait_for_sync(void)
{
}

/* sx8652_async_rx(): callback, when spi readchannels is complete */
static void sx8652_async_rx(void *ads)
{
	struct sx8652 *ts = ads;
	unsigned long flags;
	u16 *data_ptr;
	u32 rt=0;
	int i;
	int x = 0, y = 0, z1 = 0, z2 = 0;

	spin_lock_irqsave(&ts->lock, flags);

	// first byte is command of readchan
	data_ptr = (u16 *)&ts->data[1];

	for (i = 0; i < NUM_READ_REGS; i++) {
		u16 data = swab16(data_ptr[i]);
		u8 ch = data >> 12;
		printk("%d: %d\n", i, data & 0xfff);
		switch (ch) {
			case CH_X:
				x = data & 0xfff;
				break;
			case CH_Y:
				y = data & 0xfff;
				break;
			case CH_Z1:
				z1 = data & 0xfff;
				break;
			case CH_Z2:
				z2 = data & 0xfff;
				break;
			default:
				break;
		}
	}
	if (likely(y && z1)) {
		rt = z2;
		rt -= z1;
		rt *= y;
		rt *= ts->y_plate_ohms;
		rt /= z1;
		rt = (rt + 2047) >> 12;
	}
	if (rt > MAX_12BIT) {
		//dev_err(&ts->spi->dev, "ignored pressure %d\n", rt);
		goto end;
	}

	if (rt) {
		if (!ts->pen_down) {
			//printk(KERN_ERR "pendown\n");
			input_report_key(ts->input, BTN_TOUCH, 1);
			ts->pen_down = 1;
		}

		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, rt);

		input_sync(ts->input);

		dev_dbg(&ts->spi->dev, "point(%4d,%4d), pressure (%4u)\n",
			x, y, rt);
	}

end:
	ts->spi_active = 0;
	spin_unlock_irqrestore(&ts->lock, flags);

    /* kick pen up timer - to make sure it expires again(!) */
	mod_timer(&ts->penup_timer, jiffies + msecs_to_jiffies(SX8652_TS_PENUP_TIME));

}

static int __devinit sx8652_probe(struct spi_device *spi)
{
	struct sx8652 *ts;
	struct input_dev *input_dev;
	struct sx8652_platform_data	*pdata = spi->dev.platform_data;
	int err = -1;
	struct spi_message		*m;
	struct spi_transfer		*x;
	u8 txbuf[2];

	spi->irq = pdata->get_pendown_irq();

	if (!spi->irq) {
		dev_dbg(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	/* don't exceed max specified SCLK frequency */
	if (spi->max_speed_hz > 5000000) {
		dev_dbg(&spi->dev, "SCLK %d KHz?\n", spi->max_speed_hz/1000);
		return -EINVAL;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	ts = kzalloc(sizeof(struct sx8652), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, ts);

	ts->spi = spi;
	ts->input = input_dev;

	/* Send a software reset command */
	txbuf[0] = SX8652_CMD_WRITEREG | SX8652_REG_RESET;
	txbuf[1] = RESET_VALUE;
	dev_dbg(&spi->dev, "resetting SX8652 (0x%02X 0x%02X)\n",
			txbuf[0], txbuf[1]);
	err = spi_write(spi, txbuf, 2 * sizeof(u8));
	if (err < 0)
		goto err_free_mem;
	/* sx8652 nirq is momentarily asserted after software reset */
	udelay(150);

    init_timer(&ts->penup_timer);
    setup_timer(&ts->penup_timer, sx8652_ts_penup_timer_handler,
            (unsigned long)ts);

	spin_lock_init(&ts->lock);
	mutex_init(&ts->mutex);

	ts->model = pdata->model ? : 8652;
	ts->y_plate_ohms = pdata->y_plate_ohms ? : 400;
	ts->pressure_max = pdata->pressure_max ? : ~0;

	err = setup_pendown(spi, ts);
	if (err)
		goto err_free_mem;

	ts->wait_for_sync = pdata->wait_for_sync ? : null_wait_for_sync;
	//for older kernel: snprintf(ts->phys, sizeof(ts->phys), "%s/input0", spi->dev.bus_id);
	//for 2.6.32:
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&spi->dev));

	input_dev->name = "SX8652 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &spi->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X,
			pdata->x_min ? : 0,
			pdata->x_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			pdata->y_min ? : 0,
			pdata->y_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			pdata->pressure_min, pdata->pressure_max, 0, 0);

	spi->irq = pdata->get_pendown_irq();

	m =	&ts->read_msg;
	x = &ts->read_xfer;

	spi_message_init(m);

	ts->read_cmd = SX8652_CMD_READCHAN;
	x->tx_buf = &ts->read_cmd;
	x->rx_buf = ts->data;
	x->len = sizeof(ts->data);
	spi_message_add_tail(x, m);

	m->complete = sx8652_async_rx;
	m->context = ts;

	if (request_irq(spi->irq, sx8652_irq, IRQF_TRIGGER_FALLING,
			spi->dev.driver->name, ts)) {
		dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
		err = -EBUSY;
		goto err_free_mem;
	}

	dev_info(&spi->dev, "touchscreen, irq %d\n", spi->irq);

	err = sysfs_create_group(&spi->dev.kobj, &sx8652_attr_group);
	if (err)
		goto err_remove_hwmon;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr_group;

	/* Send a software reset command */
	txbuf[0] = SX8652_CMD_WRITEREG | SX8652_REG_CTRL0;
	txbuf[1] = DLY_1_1MS;		// DLY_2_3MS;	//DLY_4_5MS
	dev_dbg(&spi->dev, "ctrl0 SX8652 (0x%02X 0x%02X)\n",
			txbuf[0], txbuf[1]);
	err = spi_write(spi, txbuf, 2 * sizeof(u8));
	if (err < 0)
		goto err_remove_attr_group;

	txbuf[0] = SX8652_CMD_WRITEREG | SX8652_REG_CTRL1;
	txbuf[1] = CONDIRQ | FILT_7SA;
	dev_dbg(&spi->dev, "ctrl1 SX8652 (0x%02X 0x%02X)\n",
			txbuf[0], txbuf[1]);
	err = spi_write(spi, txbuf, 2 * sizeof(u8));
	if (err < 0)
		goto err_remove_attr_group;

	txbuf[0] = SX8652_CMD_WRITEREG | SX8652_REG_CHANMSK;
	txbuf[1] = CHAN_MASK;
	dev_dbg(&spi->dev, "chanmask SX8652 (0x%02X 0x%02X)\n",
			txbuf[0], txbuf[1]);
	err = spi_write(spi, txbuf, 2 * sizeof(u8));
	if (err < 0)
		goto err_remove_attr_group;

	txbuf[0] = SX8652_CMD_PENTRG;
	dev_dbg(&spi->dev, "pentrg SX8652 (0x%02X 0x%02X)\n",
			txbuf[0], txbuf[1]);
	err = spi_write(spi, txbuf, 1 * sizeof(u8));
	if (err < 0)
		goto err_remove_attr_group;

	return 0;

 err_remove_attr_group:
	sysfs_remove_group(&spi->dev.kobj, &sx8652_attr_group);
 err_remove_hwmon:
	free_irq(spi->irq, ts);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}


/* Must be called with ts->lock held */
static void sx8652_disable(struct sx8652 *ts)
{
	if (ts->disabled)
		return;

	disable_irq(ts->spi->irq);
	ts->disabled = 1;

	/* if timer is running, wait for it to finish */
	while (ts->pen_down) {
		msleep(5);
	}

}

static int sx8652_suspend(struct spi_device *spi, pm_message_t message)
{
	struct sx8652 *ts = dev_get_drvdata(&spi->dev);

	mutex_lock(&ts->mutex);
	sx8652_disable(ts);
	mutex_unlock(&ts->mutex);

	return 0;
}

static int __devexit sx8652_remove(struct spi_device *spi)
{
	struct sx8652		*ts = dev_get_drvdata(&spi->dev);

	input_unregister_device(ts->input);

	sx8652_suspend(spi, PMSG_SUSPEND);

	sysfs_remove_group(&spi->dev.kobj, &sx8652_attr_group);

	free_irq(ts->spi->irq, ts);
	kfree(ts);

	dev_dbg(&spi->dev, "unregistered touchscreen\n");
	return 0;
}

/* Must be called with ts->lock held */
static void sx8652_enable(struct sx8652 *ts)
{
	if (!ts->disabled)
		return;

	ts->disabled = 0;
	enable_irq(ts->spi->irq);
}

static int sx8652_resume(struct spi_device *spi)
{
	struct sx8652 *ts = dev_get_drvdata(&spi->dev);

	mutex_lock(&ts->mutex);
	sx8652_enable(ts);
	mutex_unlock(&ts->mutex);

	return 0;
}

static struct spi_driver sx8652_driver = {
	.driver = {
		.name	= "sx8652",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= sx8652_probe,
	.remove		= __devexit_p(sx8652_remove),
	.suspend	= sx8652_suspend,
	.resume		= sx8652_resume,
};

static int __init sx8652_init(void)
{
	return spi_register_driver(&sx8652_driver);
}
module_init(sx8652_init);

static void __exit sx8652_exit(void)
{
	spi_unregister_driver(&sx8652_driver);
}
module_exit(sx8652_exit);

MODULE_DESCRIPTION("SX8652 TouchScreen Driver");
MODULE_LICENSE("GPL");
