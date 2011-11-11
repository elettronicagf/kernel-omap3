/*
 * Jtag driver to program Altera MaxII CPLD using Altera
 * Jam STAPL player
 *
 * Copyright (C) 2011 ElettronicaGF s.r.l.
 * Author: Stefano Donati <stefano.donati@elettronicagf.it>
 * Author: Andrea Collamati <andrea.collamati@elettronicagf.it>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/jtag_cpld.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <linux/irqflags.h>
#include <linux/spinlock.h>
#include <linux/delay.h>	

#define TMS_PIN 141
#define TCK_PIN 142
#define TDI_PIN 140
#define TDO_PIN 143
#define TCK_HIGH 1
#define TCK_LOW 0
struct jtag_cpld_dev {
	struct cdev cdev;
	char val[3];
};

int jtag_cpld_major = 0;
int jtag_cpld_minor = 0;


MODULE_AUTHOR("A.Collamati,S.Donati");
MODULE_LICENSE("Dual BSD/GPL");

static struct jtag_cpld_dev myDev;
static struct device *jtag_dev;
static DEFINE_SPINLOCK(lock);

/*
 * Open and close
 */
int jtag_cpld_open(struct inode *inode, struct file *filp)
{
	struct jtag_cpld_dev *dev; /* device information */
	dev = container_of(inode->i_cdev, struct jtag_cpld_dev, cdev);
	filp->private_data = dev; /* for other methods */
	return 0; /* success */
}

int jtag_cpld_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/*
 * Data management: ioctl
 */
long jtag_cpld_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	char tmp;
	int retval = 0;

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != JTAG_CPLD_IOC_MAGIC) {
		dev_err(jtag_dev, "Jtag_cpld: Wrong cmd\n");
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > 0) {
		dev_err(jtag_dev, "Jtag_cpld: Wrong cmd\n");
		return -ENOTTY;
	}
	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) {
		dev_err(jtag_dev, "Jtag_cpld: permission error");
		return -EFAULT;
	}

	/* Copy data from user space */
	retval = __get_user(tmp, (char __user *)arg);

	if (retval == 0) {

		/* Write Tdi and Tms */
		char tdi = tmp & TDI_MASK;
		char tms = tmp & TMS_MASK;
		char tdo_read = tmp & TDO_READ_MASK;
		int tdo;

		unsigned long flag;

		gpio_set_value(TMS_PIN, tms);
		gpio_set_value(TDI_PIN, tdi);

		/* Read Tdo, if needed */
		if (tdo_read) {
			tdo = gpio_get_value(TDO_PIN);

			if (tdo) {
				tmp = tmp | TDO_MASK;
			} else {
				tmp = tmp & ~TDO_MASK;
			}

			retval = put_user(tmp, (char __user *)arg);
		}

		spin_lock_irqsave(&lock,flag);

		gpio_set_value(TCK_PIN, TCK_HIGH);

		gpio_set_value(TCK_PIN, TCK_LOW);

		spin_unlock_irqrestore(&lock,flag);
	} else {
		dev_err(jtag_dev, "Jtag_cpld: Unable to obtain data from user space\n");
	}

	return retval;
}

struct file_operations jtag_cpld_fops = {
		.owner = THIS_MODULE,
		.open = jtag_cpld_open,
		.release = jtag_cpld_release,
		.unlocked_ioctl =jtag_cpld_ioctl,
};

int jtag_cpld_init_module(void)
{
	int result, err, devno;
	dev_t dev = 0;
	spin_lock_init(&lock);

	/*
	 * Get a range of minor numbers to work with, asking for a dynamic
	 * major unless directed otherwise at load time.
	 */

	result = alloc_chrdev_region(&dev, jtag_cpld_minor, 1, "jtag_cpld");
	jtag_cpld_major = MAJOR(dev);

	if (result < 0) {
		return result;
	}

	/*
	 * allocate the devices -- we can't have them static, as the number
	 * can be specified at load time
	 */

	memset(&myDev, 0, sizeof(struct jtag_cpld_dev));

	devno = MKDEV(jtag_cpld_major, 0);

	cdev_init(&myDev.cdev, &jtag_cpld_fops);
	myDev.cdev.owner = THIS_MODULE;
	myDev.cdev.ops = &jtag_cpld_fops;
	myDev.val[0] = 'A';
	myDev.val[1] = 'B';
	myDev.val[2] = 'C';
	err = cdev_add(&myDev.cdev, devno, 1);
	jtag_dev=&myDev.cdev;

	/* Fail gracefully if need be */
	if (err) {
		dev_err(jtag_dev, "Error %d adding jtag_cpld\n", err );
		goto fail;
	}

	err = gpio_request(TDO_PIN, "Tdo");
	if (err != 0) {
		dev_err(jtag_dev, "Error requesting TDO pin\n");
		return 1;
	}

	err = gpio_direction_input(TDO_PIN);
	if (err != 0) {
		dev_err(jtag_dev, "Error inizializing TDO pin direction\n");
		return 1;
	}

	err = gpio_request(TDI_PIN, "Tdi");
	if (err != 0) {
		dev_err(jtag_dev, "Error requesting TDI pin\n");
		return 1;
	}

	err = gpio_direction_output(TDI_PIN, 0);
	if (err != 0) {
		dev_err(jtag_dev, "Error inizializing TDI pin direction\n");
		return 1;
	}

	err = gpio_request(TCK_PIN, "Tck");
	if (err != 0) {
		dev_err(jtag_dev, "Error requesting TCK pin\n");
		return 1;
	}

	err = gpio_direction_output(TCK_PIN, 0);
	if (err != 0) {
		dev_err(jtag_dev, "Error inizializing TCK pin direction\n");
		return 1;
	}

	err = gpio_request(TMS_PIN, "Tms");
	if (err != 0) {
		dev_err(jtag_dev, "Error requesting TMS pin\n");
		return 1;
	}
	err = gpio_direction_output(TMS_PIN, 0);
	if (err != 0) {
		dev_err(jtag_dev, "Error inizializing TMS pin direction\n");
		return 1;
	}

	return 0; /* succeed */

fail:
	return result;
}

void jtag_cpld_cleanup_module(void)
{

	dev_t devno = MKDEV(jtag_cpld_major, jtag_cpld_minor);

	cdev_del(&myDev.cdev);

	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, 1);

}

module_init(jtag_cpld_init_module);
module_exit(jtag_cpld_cleanup_module);
