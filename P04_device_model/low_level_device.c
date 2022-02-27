/* Assignment for device model */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "I2C_PLDRV"

//TODO 5.6: Intialize the start address and end address for I2C 0
#define RESOURCE1_START_ADDRESS
#define RESOURCE1_END_ADDRESS

/* Specifying my resources information */
//TODO 5.7: Populate the memory resource
static struct resource sample_resources[] = {
	{
	},
};

//TODO 5.8: Define and initialize the clock frequence to 400KHz

//TODO 5.9: Populate the platform device structure
static struct platform_device sample_device = 
{
};

static __init int init_platform_dev(void)
{
	printk("sample Platform driver(device).... \n");
	//TODO 5.10: Register the platform device
	return 0;
}

static void __exit exit_platform_dev(void)
{
	//TODO 5.11: Un-register the platform device
	printk("Exiting sample Platform(device) driver... \n");
}

module_init(init_platform_dev);
module_exit(exit_platform_dev);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Sample Platform Driver");
MODULE_LICENSE("GPL");
