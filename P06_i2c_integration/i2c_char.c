#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_data/serial-omap.h>
#include <linux/i2c.h>

static int my_open(struct inode *i, struct file *f)
{
	return 0;
}

static int my_close(struct inode *i, struct file *f)
{
	return 0;
}

static ssize_t my_read(struct file *f, char __user *buf, size_t count, loff_t *off)
{
	struct i2c_msg msg[2];
	int i;
	u8 buff[6] = {0x00, 0x60, 0xaa};
	memset(msg, 0, sizeof(msg));
	/* 
	 * TODO: Since read invokes i2c transmit and i2c receive,
	 * there is a need to send 2 i2c_msg - one for transmit
	 * and other for receive.
	 * Initialize the fields for both i2c_msg & invoke the 
	 * low level driver API to transfer over the I2C bus
	 */
	return 0;
}

static ssize_t my_write(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
	struct i2c_msg msg;
	u8 buff[3] = {0x00, 0x60, 0xaa};

	memset(&msg, 0, sizeof(msg));
	/* 
	 * TODO: Initialize the i2c_msg fields & invoke the low level
	 * driver API to transfer over the I2C bus
	 */
	return count;
}

static struct file_operations driver_fops =
{
	.owner = THIS_MODULE,
	.open = my_open,
	.release = my_close,
	.read = my_read,
	.write = my_write
};

int chrdrv_init(struct omap_i2c_dev *i2c_dev)
{
	/* Same as previous assignment */
	return 0;
}

void chrdrv_exit(struct omap_i2c_dev *i2c_dev)
{
	/* Same as previous assignment */
}

