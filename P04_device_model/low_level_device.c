/* Assignment for device model */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "Sample_Pldrv"

//TODO: Intialize the start address and end address for I2C 0
#define RESOURCE1_START_ADDRESS
#define RESOURCE1_END_ADDRESS

/* Specifying my resources information */
//TODO: Populate the memory resource
static struct resource sample_resources[] = {
	{
	},
};

//TODO: Define and initialize the clock frequence to 400KHz

//TODO: Populate the platform device structure
static struct platform_device sample_device = 
{
};

static __init int init_platform_dev(void)
{
	printk("sample Platform driver(device).... \n");
	//TODO: Register the platform device
	return 0;
}

static void __exit exit_platform_dev(void)
{
	//TODO: Un-register the platform device
	printk("Exiting sample Platform(device) driver... \n");
}

module_init(init_platform_dev);
module_exit(exit_platform_dev);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Sample Platform Driver");
MODULE_LICENSE("GPL");
