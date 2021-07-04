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
	// TODO: Add the code here
	i2c_transmit(NULL, 0);
	i2c_receive(NULL, 0);
	return 0;
}

static ssize_t my_write(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
	// TODO: Add the code here
	i2c_transmit(NULL, 0);
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

	// TODO: Create the device file
	if (device_create(i2c_dev->i2c_class, NULL, i2c_dev->devt, NULL, "i2c_drv%d", i2c_num++) == NULL)
	{
		printk( KERN_ALERT "Device creation failed\n" );
		unregister_chrdev_region(i2c_dev->devt, 1);
		return -1;
	}

	// TODO: Register the file_operations
	cdev_init(&i2c_dev->cdev, &driver_fops);

	if (cdev_add(&i2c_dev->cdev, i2c_dev->devt, 1) == -1)
	{
		printk( KERN_ALERT "Device addition failed\n" );
		device_destroy(i2c_dev->i2c_class, i2c_dev->devt);
		unregister_chrdev_region(i2c_dev->devt, 1);
		return -1;
	}
	return 0;
}

void chrdrv_exit(struct omap_i2c_dev *i2c_dev)
{
	// TODO: Delete the device file
	device_destroy(i2c_dev->i2c_class, i2c_dev->devt);
	// TODO: Unregister file operations
	cdev_del(&i2c_dev->cdev);
	// TODO: Unregister character driver
	unregister_chrdev_region(i2c_dev->devt, 1);
}

