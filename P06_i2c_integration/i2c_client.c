#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_data/serial-omap.h>
#include "i2c_char.h"

struct chrdev_data {
	struct i2c_client *client;
	u8 *write_buff;
	u16 write_max;
	/* Character Driver Files */
	dev_t devt;
	struct cdev cdev;
	struct class *class;
};	

static int my_open(struct inode *i, struct file *f)
{
	struct chrdev_data *dev = container_of(i->i_cdev, struct chrdev_data, cdev);
	if (dev == NULL) {
		printk("Data is null\n");
		return -1;
	}
	f->private_data = dev;

	return 0;
}

static int my_close(struct inode *i, struct file *f)
{
	return 0;
}

static ssize_t my_read(struct file *f, char __user *buf, size_t count, loff_t *off)
{
	struct i2c_msg msg[2];
	u8 buff[6] = {0x00, 0x60, 0xaa};
	int i, ret = 0;
	struct i2c_adapter *adap;
	struct i2c_client *client;

	/*
	 * TODO 9.14: Get chrdev_data from private data field and 
	 * correspondingly get i2c_adapter and i2c_client to 
	 * be used in subsequent APIs
	 */
	memset(msg, 0, sizeof(msg));

	/* 
	 * TODO 8.8: Since read invokes i2c transmit and i2c receive,
	 * there is a need to send 2 i2c_msg - one for transmit
	 * and other for receive.
	 * Initialize the fields for both i2c_msg 
	 * msg.buff, msg.len and msg.addr for msg[0] and msg[1]
	 * Use I2C_M_RD flag for msg[1]
	 */

	//TODO 9.15: Get the slave address from i2c_client data structure

	printk("Initiatiating Transaction\n");

	//TODO 9.16 Invoke the i2c_transfer
	if (ret < 0) {
		printk("Error in reception\n");
		return -1;
	}
	printk("The data is:\n");

	for (i = 0; i < 3; i++) {
	       printk("%x\t", msg[1].buf[i]);
	}

	return 0;
}

static ssize_t my_write(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
	struct i2c_msg msg;
	u8 buff[3] = {0x00, 0x30, 0xaa};
	int ret = 0;

	/*
	 * TODO 9.11: Get chrdev_data from private data field and 
	 * correspondingly get i2c_adapter and i2c_client to 
	 * be used in subsequent APIs
	 */

	memset(&msg, 0, sizeof(msg));
	/* 
	 * TODO 9.12: Initialize the i2c_msg fields & invoke the
	 * i2c_transfer API to transfer over the I2C bus
	 */

	//TODO 9.13: Invoke i2c_transfer for 1 message
	// Takes 3 arguments - adapter, msg and count of messages
	if (ret < 0) {
		printk("Error in transmission\n");
		return -1;
	}

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

int chrdev_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct chrdev_data *data;
	int init_result;

	printk("Client Driver probe Invoked\n");
	data = devm_kzalloc(&client->dev, sizeof(struct chrdev_data), GFP_KERNEL);
	data->write_max = 32;
	data->write_buff = devm_kzalloc(&client->dev, data->write_max, GFP_KERNEL);
	i2c_set_clientdata(client, data);
	init_result = alloc_chrdev_region(&(data->devt), 0, 1, "i2c_drv");
	data->client = client;

	if (0 > init_result)
	{
		printk(KERN_ALERT "Device Registration failed\n");
		unregister_chrdev_region(data->devt, 1);
		return -1;
	}
	printk("Major Nr: %d\n", MAJOR(data->devt));

	if ((data->class = class_create(THIS_MODULE, "i2c_char")) == NULL)
	{
		printk("Class creation failed\n");
		unregister_chrdev_region(data->devt, 1);
		return -1;
	}

	if (device_create(data->class, NULL, data->devt, NULL, "i2c_drv%d", 0) == NULL)
	{
		printk( KERN_ALERT "Device creation failed\n" );
		unregister_chrdev_region(data->devt, 1);
		return -1;
	}

	cdev_init(&data->cdev, &driver_fops);

	if (cdev_add(&data->cdev, data->devt, 1) == -1)
	{
		printk( KERN_ALERT "Device addition failed\n" );
		device_destroy(data->class, data->devt);
		class_destroy(data->class);
		unregister_chrdev_region(data->devt, 1);
		return -1;
	}
	return 0;
}

static int chrdev_remove(struct i2c_client *client)
{
	struct chrdev_data *dev;
	printk("Remove Invoked\n");
	dev = i2c_get_clientdata(client);
	cdev_del(&dev->cdev);
	device_destroy(dev->class, dev->devt);
	class_destroy(dev->class);
	unregister_chrdev_region(dev->devt, 1);
	return 0;
}

// TODO 9.7: Populate the id table to expose the devices supported
static const struct i2c_device_id chrdev_ids[] = {
	{ },
	{ } // Don't delete this. Serves as terminator
};
MODULE_DEVICE_TABLE(i2c, chrdev_ids);

// TODO 9.8: Populate the i2c_driver structure
// Add .probe, .remove and .id_table
static struct i2c_driver chrdev_driver = {
};

static int __init chrdev_init(void)
{
	// TODO 9.9: Register the client driver
	return 0;
}

static void __exit chrdev_exit(void)
{
	// TODO 9.10: De-register the client driver
}

module_init(chrdev_init)
module_exit(chrdev_exit)

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Dummy Client Driver");
MODULE_LICENSE("GPL");
