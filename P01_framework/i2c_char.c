#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_data/serial-omap.h>
#include "i2c_char.h"

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
	// TODO 4.11: Invoke i2c_transmit from low level driver
	// TODO 1.10: Invoke i2c_receive from low level driver

	/* 
	 * TODO 8.8: Since read invokes i2c transmit and i2c receive,
	 * there is a need to send 2 i2c_msg - one for transmit
	 * and other for receive.
	 * Initialize the fields for both i2c_msg
	 * msg.buff, msg.len and msg.addr for msg[0] and msg[1]
	 * Use I2C_M_RD flag for msg[1]
	 */

	printk("Initiatiating Transaction\n");
	//TODO 8.9 Invoke the i2c_txrx for 2 messages

	return 0;
}

static ssize_t my_write(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
	// TODO 1.11: Invoke i2c_transmit() from low level driver

	//TODO 8.6: Initialize the i2c_msg fields

	//TODO 8.7: Invoke i2c_txrx for 1 message
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
	static int i2c_num = 0;
	int init_result = alloc_chrdev_region(&i2c_dev->devt, 0, 1, "i2c_drv");

	if (0 > init_result)
	{
		printk(KERN_ALERT "Device Registration failed\n");
		return -1;
	}
	printk("Major Nr: %d\n", MAJOR(i2c_dev->devt));

	// TODO 1.4: Create the class with name i2cdrv

	// TODO 1.5: Create the device file

	// TODO 1.6: Register the file_operations
	return 0;
}

void chrdrv_exit(struct omap_i2c_dev *i2c_dev)
{
	// TODO 1.7: Delete the device file
	// TODO 1.3: Delete the i2cdrv class
	// TODO 1.8: Unregister file operations
	// TODO 1.9: Unregister character driver
}
