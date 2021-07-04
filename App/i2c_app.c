#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include "i2c-dev.h"


int main()
{
#if 0 // Method 1
	uint8_t val;
	int ret;
	uint8_t ee_addr[2] = {0x00, 0x60};
	uint8_t ee_data[3];
	int fd = open("/dev/i2c-3", O_RDWR);

	if (ioctl(fd, I2C_SLAVE_FORCE, 0x50) < 0)
		return -1;
	if ((ret = write(fd, ee_addr, 2)) < 0) {
		printf("Write failed with ret = %d\n", ret);	
		return -1;
	}

	if (read(fd, ee_data, 3) < 0) 
		return -1;
	printf("data = %x, %x, %x\n", ee_data[0], e_data[1], ee_data[2]);

#else //Method 2
	struct i2c_rdwr_ioctl_data i2c_data;
	struct i2c_msg msg[2];
	uint8_t ee_addr[2] = {0x00, 0x60};
	uint8_t ee_data[3];

	int fd = open("/dev/i2c-3", O_RDWR);

	i2c_data.nmsgs = 2;
	i2c_data.msgs = msg;

	i2c_data.msgs[0].addr = 0x50;
	i2c_data.msgs[0].len = 2;
	i2c_data.msgs[0].flags = 0;
	i2c_data.msgs[0].buf = ee_addr;

	i2c_data.msgs[1].addr = 0x50;
	i2c_data.msgs[1].len = 3;
	i2c_data.msgs[1].flags = I2C_M_RD;
	i2c_data.msgs[1].buf = ee_data;

	if (ioctl(fd, I2C_RDWR, &i2c_data) < 0) {
		printf("Error in reading\n");
		return -1;
	}
	printf("data = %x, %x, %x\n", msg[1].buf[0], msg[1].buf[1], msg[1].buf[2]);
#endif
	return 0;
}
