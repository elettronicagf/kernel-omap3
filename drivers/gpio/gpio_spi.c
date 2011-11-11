/*
 * SPI Gpio Expander based on MAXII Altera CPLD used in
 * elettronica GF SOM336 board
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
#ifndef DEBUG
#define DEBUG
#endif
#include <linux/device.h>
#include <linux/platform_device.h>
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
#include <linux/gpio_spi.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <linux/irqflags.h>
#include <linux/spinlock.h>	
#include <linux/jiffies.h>
#include <linux/spi/spi.h>
#include <linux/timer.h>

MODULE_AUTHOR("S.Donati, A.Collamati");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("SPI GPIO Expander Driver");

#define DRIVER_NAME "gpio_spi"

#define START_OUTPUTS_OFFSET0	 0
#define STOP_OUTPUTS_OFFSET0 	 25
#define START_INPUTS_OFFSET0     26
#define STOP_INPUTS_OFFSET0      51

/* Input/Output Gpio Map
 *
 * Outputs from  GPIO212 to GPIO237
 * Inputs  from  GPIO238 to GPIO265
 *
 */

#define N_OUTPUTS 26
#define N_INPUTS 26
#define N_GPIO 52
#define GPIO_BASE 212
#define BIT_PER_WORD 32
#define SPI_MAX_SPEED 300000

struct gpio_spi {
	struct cdev cdev;
	struct gpio_chip chip;
	struct spi_device *spi;
	struct platform_device	*pdev;
	u32 output_reg;
	u32 input_reg;
  	u32 rx_buf;
	u32 tx_buf;
	int cpld_sw_version;
};

struct gpio_spi_packet {
  	struct spi_message message;
  	struct spi_transfer xfer;
};

static DEFINE_MUTEX(lock);

static int gpio_spi_major =   0;
static int gpio_spi_minor =   0;
static struct gpio_spi_packet packet;
static struct gpio_spi myDev;
static struct device *gpio_spi_dev;

int gpio_spi_open(struct inode *inode, struct file *filp)
{
	struct gpio_spi *dev;

	dev = container_of(inode->i_cdev, struct gpio_spi, cdev);
	filp->private_data = dev;

	return 0;
}

int gpio_spi_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int gpio_spi_sync (struct gpio_chip *chip, int readOnlyOutput)
{
	int err;
	if(!readOnlyOutput){
		/* Write 0 in output_reg's MSB => CPLD in update I/O mode */
		myDev.tx_buf = myDev.output_reg & 0x7FFFFFFF;
		err=spi_sync(myDev.spi,&packet.message);
		if(err < 0)
			dev_err(gpio_spi_dev, "Gpio_spi: Errore durante la comunicazione spi con la cpld %d \n",err);
		
		myDev.input_reg = myDev.rx_buf;
	}else{
		/* Write 1 in output_reg's MSB => CPLD in reading output mode */
		myDev.tx_buf = myDev.output_reg | 0x80000000;
		err=spi_sync(myDev.spi, &packet.message);
		if(err < 0){
			dev_err(gpio_spi_dev, "Gpio_spi: Errore durante la comunicazione spi con la cpld %d \n",err);
		}else{
		  myDev.output_reg = myDev.rx_buf;
		}
	}
	return err;
}

static int gpio_spi_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static int gpio_spi_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	return 0;
}
static int is_input(unsigned offset)
{
  return (offset >= START_INPUTS_OFFSET0 && offset <= STOP_INPUTS_OFFSET0);
}

static void gpio_spi_set(struct gpio_chip *chip, unsigned offset, int value)
{
	mutex_lock(&lock);
	if(is_input(offset)){
		dev_err(gpio_spi_dev, "gpio_spi: Error! Trying to write an input\n");
	}else{
		int pos = (offset - START_OUTPUTS_OFFSET0);
		if(value)
			myDev.output_reg |= (1 << pos);
		else 
			myDev.output_reg &= ~(1 << pos);
	}
	gpio_spi_sync(chip,0);
	mutex_unlock(&lock);
}

static int gpio_spi_get(struct gpio_chip *chip, unsigned offset)
{
	u32 tmp;
	int pos;
	mutex_lock(&lock);
	if(is_input(offset)){
		/*Reading inputs*/
		gpio_spi_sync(chip, 0);
		pos = (offset - START_INPUTS_OFFSET0);
		tmp = myDev.input_reg & (1 << pos);
	}else{	
		/*Reading outputs*/
		gpio_spi_sync(chip, 1);
		pos = (offset - START_OUTPUTS_OFFSET0);
		tmp = myDev.output_reg & (1 << pos);
	}
	mutex_unlock(&lock);
	return (tmp);

}
	
static long gpio_spi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{	
	int retval=0;
	u32 tmp;

	mutex_lock(&lock);

	if (_IOC_TYPE(cmd) != GPIO_SPI_IOC_MAGIC)  {
		dev_err(gpio_spi_dev, "Gpio_spi: ioctl: Wrong cmd1\n");
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > 2) {
		dev_err(gpio_spi_dev, "Gpio_spi: ioctl: Wrong cmd\n");
		return -ENOTTY;
	}
	retval = __copy_from_user(&tmp, (u32 __user *) arg, 2);
	
	if (!retval){
		switch(cmd) {
		case GPIO_SPI_READ_INPUTS:
			gpio_spi_sync(&myDev.chip,0);
			tmp = myDev.input_reg;
			retval = __copy_to_user( (u32 __user *) arg, &tmp, 2);
			break;
		case GPIO_SPI_WRITE_OUTPUTS:
			myDev.output_reg = (u32) (tmp & 0xFFFFFFFF);
			gpio_spi_sync(&myDev.chip,0);
			break;
		case GPIO_SPI_READ_OUTPUTS:
			gpio_spi_sync(&myDev.chip,1);
			tmp = myDev.output_reg;
			retval = __copy_to_user( (u64 __user *) arg, &tmp, 2);
			break;
		default:
			retval=-1;
		}


	}else{
		dev_err(gpio_spi_dev, "Gpio_spi: ioctl: unable to obtain data from user space\n");
	}
	 mutex_unlock(&lock);
	return retval;
}


static struct file_operations gpio_spi_fops = {
	.owner =    THIS_MODULE,
	.open =     gpio_spi_open,
	.release =  gpio_spi_release,
	.unlocked_ioctl = gpio_spi_ioctl,
};

/* init gpio_chip */
static int gpio_spi_init_gpiochip(struct gpio_chip* chip, struct spi_device* dev)
{
	int ret;
	chip->label=DRIVER_NAME;
	chip->direction_input=gpio_spi_direction_input;
	chip->direction_output=gpio_spi_direction_output;
	chip->set=gpio_spi_set;
	chip->get =gpio_spi_get;
	chip->base=GPIO_BASE;
	chip->ngpio=N_GPIO;
	chip->can_sleep=0;
	chip->dev=(struct device*) dev;
	chip->owner= THIS_MODULE;
	ret = gpiochip_add(chip);
	return ret;
}

/* init spi device */
static int gpio_spi_init_spi(struct spi_device *spi)
{
	int ret;
	spi->bits_per_word = BIT_PER_WORD;
	spi->mode          = SPI_MODE_3;
	spi->max_speed_hz  = SPI_MAX_SPEED;
	ret=spi_setup(spi);
	myDev.spi=spi;
	return ret;
}

/* spi_message and spi_transfer init */
static void gpio_spi_init_message(void)
{
	struct spi_message *m;
	struct spi_transfer *x;

	m = &packet.message;
	spi_message_init(m);
	m->spi=myDev.spi;
	m->is_dma_mapped=0;

	x = &packet.xfer;
	memset (x,0,sizeof(*x));
	x->tx_buf    = &myDev.tx_buf;
	x->rx_buf    = &myDev.rx_buf;
	x->len       = BIT_PER_WORD/8;
	x->cs_change = 0;
	spi_message_add_tail(x,m);

}

/* init character device*/
static int gpio_spi_init_cdev(void)
{
	int ret,devno;
	dev_t dev = 0;
	ret = alloc_chrdev_region(&dev, gpio_spi_minor, 1 ,"gpio_spi");
	gpio_spi_major = MAJOR(dev);
	memset(&myDev.cdev, 0, sizeof(struct cdev));
	devno = MKDEV(gpio_spi_major, 0);
	cdev_init(&myDev.cdev, &gpio_spi_fops);
	myDev.cdev.owner = THIS_MODULE;
	cdev_add (&myDev.cdev, devno, 1);
	return ret;
}

static int gpio_spi_load_outputs(void)
{
	int ret;
	mutex_lock(&lock);
	ret=gpio_spi_sync(&myDev.chip,1);
	mutex_unlock(&lock);
	return ret;
	
}


static int load_cpld_sw_version(void)
{
	int ret,response;
	int major, minor;
	mutex_lock(&lock);
	ret=gpio_spi_sync(&myDev.chip,0);
	if(ret)
		return ret;
	response = myDev.input_reg;
	mutex_unlock(&lock);

	major = (response >> 22) & 0x0F;
	minor = (response >> 14) & 0xFF;
	myDev.cpld_sw_version = major * 100 + minor;
	return 0;
}

int get_cpld_sw_version(void)
{
	return myDev.cpld_sw_version;
}


/* Start Sysfs stuff */
static ssize_t gpio_spi_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int v = get_cpld_sw_version();
	return sprintf(buf, "%d\n", v);
}


static DEVICE_ATTR(cpld_sw_version, S_IRUGO | S_IWUSR, gpio_spi_show,NULL);

static struct attribute *gpio_spi_attributes[] = {
	&dev_attr_cpld_sw_version.attr,
	NULL
};

static struct attribute_group gpio_spi_attribute_group = {
	.attrs = gpio_spi_attributes
};


static int gpio_spi_add_fs(void)
{
	myDev.pdev = platform_device_register_simple("gpio_spi", -1, NULL, 0);
	if (IS_ERR(myDev.pdev))
		return PTR_ERR(myDev.pdev);
	return sysfs_create_group(&myDev.pdev->dev.kobj, &gpio_spi_attribute_group);
}

int gpio_spi_remove_fs(void)
{
	sysfs_remove_group(&myDev.pdev->dev.kobj, &gpio_spi_attribute_group);
	return 0;
}
EXPORT_SYMBOL_GPL(gpio_spi_remove_fs);
/* End of sysfs stuff*/

static int __devinit gpio_spi_probe(struct spi_device *spi)
{
	int ret;

	memset(&packet,0,sizeof(packet));

	ret=gpio_spi_init_spi(spi);
	if(ret<0)
		return ret;

	gpio_spi_init_message();
	
	ret = gpio_spi_init_gpiochip(&myDev.chip,spi);
	if(ret)
		return ret;

	ret=gpio_spi_init_cdev();
	if(ret)
		return ret;

	ret=gpio_spi_load_outputs();
	if(ret)
		return ret;

	gpio_spi_add_fs();

	load_cpld_sw_version();

	gpio_spi_dev=&myDev.pdev->dev;

	dev_dbg(gpio_spi_dev,"Gpio_spi: CPLD_VERSION %d\n", myDev.cpld_sw_version);

	return ret;
}


static int gpio_spi_remove (struct spi_device *spi)
{
	int ret;
	ret=gpiochip_remove(&myDev.chip);
	if(ret)
		dev_err(gpio_spi_dev, "Gpio_spi: Failed to remove the gpio_chip controller %d \n",ret);

	gpio_spi_remove_fs();
	return ret;
}

static struct spi_driver gpio_spi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= gpio_spi_probe,
	.remove		= __devexit_p(gpio_spi_remove),
};

static int __init gpio_spi_init(void)
{
	return spi_register_driver(&gpio_spi_driver);
}



static void __exit gpio_spi_exit(void)
{
	dev_t devno;
	spi_unregister_driver(&gpio_spi_driver);
	devno = MKDEV(gpio_spi_major, gpio_spi_minor);
	cdev_del(&myDev.cdev);
	unregister_chrdev_region(devno, 1);

}

subsys_initcall(gpio_spi_init);
module_exit(gpio_spi_exit);
