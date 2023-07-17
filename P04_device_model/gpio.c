#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>

#define GPIO_NUMBER 56

static dev_t first;			// Global variable for the first device number
static struct cdev c_dev;	// Global variable for the character device structure
static struct class *cl;	// Global variable for the device class

static ssize_t gpio_read(struct file* F, char *buf, size_t count, loff_t *f_pos)
{
	unsigned char temp; 
	//TODO 1: Get the led status in temp 
	temp = gpio_get_value(GPIO_NUMBER);
	if (copy_to_user(buf, &temp, 1))
	{
		return -EFAULT;
	}
	return count;
}

static ssize_t gpio_write(struct file* F, const char *buf, size_t count, loff_t *f_pos)
{
	char temp;

	if (copy_from_user(&temp, buf, count))
	{
		return -EFAULT;
	}

	printk(KERN_INFO "Executing WRITE.\n");
	// TODO 2: Switch On the LED for value of '1' and switch it off for value of '0'
	//53, 54, 55, 56
	
	gpio_set_value(GPIO_NUMBER, temp - '0');

	return count;
}

static int gpio_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int gpio_close(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations file_ops =
{
	.owner		= THIS_MODULE,
	.open		= gpio_open,
	.read		= gpio_read,
	.write		= gpio_write,
	.release	= gpio_close,
};

static int init_gpio(void)
{
	int ret;
	struct device *dev_ret;

	if ((ret = alloc_chrdev_region(&first, 0, 1, "gpio_drv")) < 0)
	{
		printk(KERN_ALERT "Device registration failed\n");
		return ret;
	}
	printk("Major Nr: %d\n", MAJOR(first));

	if (IS_ERR(cl = class_create(THIS_MODULE, "gpiodrv")))
	{
		printk(KERN_ALERT "Class creation failed\n");
		unregister_chrdev_region(first, 1);
		return PTR_ERR(cl);
	}

	if (IS_ERR(dev_ret = device_create(cl, NULL, first, NULL, "gpio_drv%d", 0)))
	{
		printk(KERN_ALERT "Device creation failed\n");
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return PTR_ERR(dev_ret);
	}

	cdev_init(&c_dev, &file_ops);

	if ((ret = cdev_add(&c_dev, first, 1)) < 0)
	{
		printk(KERN_ALERT "Device addition failed\n");
		device_destroy(cl, first);
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return ret;
	}

	return 0;
}

void cleanup_gpio(void)
{
	cdev_del(&c_dev);
	device_destroy(cl, first);
	class_destroy(cl);
	unregister_chrdev_region(first, 1);

	printk(KERN_INFO "Device unregistered\n");
}

module_init(init_gpio);
module_exit(cleanup_gpio);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("GPIO Demo");
