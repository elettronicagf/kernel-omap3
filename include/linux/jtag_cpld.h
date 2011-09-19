#ifndef _JTAG_CPLD_H_
#define _JTAG_CPLD_H_

#define TMS_MASK 0x01
#define TDI_MASK 0x02
#define TDO_MASK 0x04
#define TDO_READ_MASK 0x08

#define JTAG_CPLD_IOC_MAGIC  'g'
#define JTAG_CPLD_CMD  _IOWR(JTAG_CPLD_IOC_MAGIC, 0, char)

#endif
