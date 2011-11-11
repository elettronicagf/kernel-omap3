#ifndef _GPIO_SPI_H
#define _GPIO_SPI_H

#define GPIO_SPI_IOC_MAGIC  'g'
#define GPIO_SPI_WRITE_OUTPUTS  _IOWR(GPIO_SPI_IOC_MAGIC, 0, __u64)
#define GPIO_SPI_READ_INPUTS  _IOR(GPIO_SPI_IOC_MAGIC, 1, __u64)
#define GPIO_SPI_READ_OUTPUTS  _IOR(GPIO_SPI_IOC_MAGIC,2, __u64)

struct gpio_spi_platform_data {
	unsigned base;
};

#endif