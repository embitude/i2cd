#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/platform_data/serial-omap.h>

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
	u8 buff[20] = {0x00, 0x60, 0xaa};
	int i;

	/*
	 * TODO: Get chrdev_data from private data field and 
	 * correspondingly get i2c_adapter and i2c_client to 
	 * be used in subsequent APIs
	 */
	struct chrdev_data *dev = (struct chrdev_data *)(f->private_data);
	struct i2c_adapter *adap = dev->client->adapter;
	struct i2c_client *client = dev->client;

	/* 
	 * TODO: Since read invokes i2c transmit and i2c receive,
	 * there is a need to send 2 i2c_msg - one for transmit
	 * and other for receive.
	 * Initialize the fields for both i2c_msg & invoke the 
	 * i2c_transfer to initiate the transaction on the bus
	 */

	memset(msg, 0, sizeof(msg));

	msg[0].buf = buff;
	msg[0].len = 2;
	msg[0].addr = client->addr;

	msg[1].buf = buff;
	msg[1].len = 20;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;

	printk("Invoking Transfer\n");
	i2c_transfer(adap, msg, 2); 
	printk("The data is:\n");
	for (i = 0; i < 20; i++) {
	       printk("%x\t", msg[1].buf[i]);
	}

	return 0;
}

static ssize_t my_write(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
	struct i2c_msg msg;
	u8 buff[20] = {0x00, 0x60, 0xaa};
	struct chrdev_data *dev = (struct chrdev_data *)(f->private_data);
	struct i2c_adapter *adap = dev->client->adapter;
	struct i2c_client *client = dev->client;

	memset(&msg, 0, sizeof(msg));

	/* 
	 * TODO: Initialize the i2c_msg fields & invoke the
	 * i2c_transfer API to transfer over the I2C bus
	 */
	msg.buf = buff;
	msg.len = sizeof(buff);
	msg.addr = client->addr;

	if (i2c_transfer(adap, &msg, 1) < 0) {
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

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,11,0))
static int chrdev_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
static int chrdev_probe(struct i2c_client *client)
#endif
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

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,4,0))
	if ((data->class = class_create(THIS_MODULE, "i2c_char")) == NULL)
#else
	if ((data->class = class_create("i2c_char")) == NULL)
#endif
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

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,11,0))
static int chrdev_remove(struct i2c_client *client)
#else
static void chrdev_remove(struct i2c_client *client)
#endif
{
	struct chrdev_data *dev;
	printk("Remove Invoked\n");
	dev = i2c_get_clientdata(client);
	cdev_del(&dev->cdev);
	device_destroy(dev->class, dev->devt);
	class_destroy(dev->class);
	unregister_chrdev_region(dev->devt, 1);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,11,0))
	return 0;
#endif
}

// TODO: Populate the id table to expose the devices supported
static const struct i2c_device_id chrdev_ids[] = {
	{ "chrdev_device", 0},
	{ } // Don't delete this. Serves as terminator
};
MODULE_DEVICE_TABLE(i2c, chrdev_ids);

// TODO: Populate the i2c_driver structure
static struct i2c_driver chrdev_driver = {
	.driver = {
		.name = "chrdv_client",
		.owner = THIS_MODULE,
	},
	.probe = chrdev_probe,
	.remove = chrdev_remove,
	.id_table = chrdev_ids,
};

static int __init chrdev_init(void)
{
	// TODO: Register the dummy client driver
	return i2c_add_driver(&chrdev_driver);
}

static void __exit chrdev_exit(void)
{
	// TODO: De-register the I2c client driver
	i2c_del_driver(&chrdev_driver);
}

module_init(chrdev_init)
module_exit(chrdev_exit)

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Dummy Client Driver");
MODULE_LICENSE("GPL");
