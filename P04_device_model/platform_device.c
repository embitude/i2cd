#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "Sample_Pldrv"

#define RESOURCE1_START_ADDRESS 0x100000
#define RESOURCE1_END_ADDRESS 0x1FFFFF
#define RESOURCE2_START_ADDRESS 0x300000
#define RESOURCE2_END_ADDRESS 0x3FFFFF
#define SAMPLE_DEV_IRQNUM 6

/* Specifying my resources information */
static struct resource sample_resources[] = {
	{
		.start = RESOURCE1_START_ADDRESS,
		.end = RESOURCE1_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = RESOURCE2_START_ADDRESS,
		.end = RESOURCE2_END_ADDRESS,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = SAMPLE_DEV_IRQNUM,
		.end = SAMPLE_DEV_IRQNUM,
		.flags = IORESOURCE_IRQ,
	}
};

int gpio = 53;
static struct platform_device sample_device = 
{
	.name = DRIVER_NAME,
	.id = -1,
	.num_resources = ARRAY_SIZE(sample_resources),
	.resource = sample_resources,
	.dev = {
		.platform_data = &gpio,
	}
};

static __init int init_platform_dev(void)
{
	printk(KERN_ALERT "\n Welcome to sample Platform driver(device).... \n");
	platform_device_register(&sample_device);
	return 0;
}

static void __exit exit_platform_dev(void)
{
	platform_device_unregister(&sample_device);
	printk(KERN_ALERT "\n Thanks....Exiting sample Platform(device) driver... \n");
}

module_init(init_platform_dev);
module_exit(exit_platform_dev);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Sample Platform Driver");
MODULE_LICENSE("GPL");
