#include "gf_eeprom_port.h"
#include "gf_eeprom.h"

static char eeprom_content[EEPROM_MAX_N_BLOCKS*EEPROM_BLOCK_LEN+1];

struct wid_translation legacy_wid_translation_table[] = {
		{"0336_A01","WID0336_AA01.00"},
		{"0336_B01","WID0336_AA01.00"},
		{"0336_C01","WID0336_AA01.00"},
		{"0336_D01","WID0336_AA01.00"},
		{"0336_E01","WID0336_AA01.00"},
		{"0336_F01","WID0336_AB01.00"},
		{"0336_F02","WID0336_AB01.00"},
		{"0336_H01","WID0336_AA01.00"},
};

void gf_i2c_init(void)
{
	/* Already done in bootloader. Not needed here */
}

int gf_i2c_set_bus_num(unsigned int bus)
{
	/* Eeprom is accessed via memory accessor in kernel
	 * This check is not needed
	 */
	return 0;
}

int gf_i2c_probe (u8 chip)
{
	/* Eeprom is accessed via memory accessor in kernel
	 * This check is not needed
	 */
	return 0;
}

int gf_serial_getc(void)
{
	/* Not used in kernel */
	return 0;
}

void gf_serial_init(void)
{
	/* Not used in kernel */
	return;
}

void gf_som_eeprom_unlock(void)
{
	/* Not needed in kernel */
}

void gf_som_eeprom_lock(void)
{
	/* Not needed in kernel */
}

int gf_eeprom_read(u8 address,u16 start_address,u8 * buffer,int len)
{
	int i;
	for(i=start_address; i<start_address+len; i++){
		if (eeprom_content[i] == 0)
		{
			buffer[i-start_address]=0;
			return 0;
		}
		buffer[i-start_address] = eeprom_content[i];
	}
	return 0;
}

int gf_eeprom_write(u8 address,u16 start_address,u8 * buffer,int len)
{
	/* EEPROM Write not needed in kernel */
	return 0;
}

void load_eeprom_content(struct memory_accessor *mem_acc, void *context)
{
	char *content =eeprom_content;
	off_t offset = (off_t)context;

	/* Read MAC addr from EEPROM */
	if (mem_acc->read(mem_acc, content, offset, EEPROM_BLOCK_LEN) == EEPROM_BLOCK_LEN)
		eeprom_content[EEPROM_BLOCK_LEN]=0;
		gf_debug(2,"Read EEPROM:\n%s\n",content);
}

int gf_read_programmer_file(const char * file_name,char * file_buffer,int buffer_length)
{
	/* EEPROM programming not needed in kernel */
	return 0;
}
