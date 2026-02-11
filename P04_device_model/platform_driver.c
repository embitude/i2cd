#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "Sample_Pldrv"

static int sample_drv_probe(struct platform_device *pdev) 
{
	struct resource *res1, *res2;
	int gpio_number;

	printk("Platform Driver Probe called\n");
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res1) 
	{
		pr_err(" Specified Resource Not Available... 1\n");
		return -1;
	}
	printk("Memory Area1\n");
	printk(KERN_ALERT "Start:%lx, End:%lx Size:%d\n", (unsigned long)res1->start, 
			(unsigned long)res1->end, resource_size(res1));
	res2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if ((!res2)) {
		pr_err(" Specified Resource Not Available... 2\n");
		return -1;
	}
	printk("Memory Area2\n");
	printk("Start:%lx, End:%lx, size:%d\n", (unsigned long)res2->start, 
			(unsigned long)res2->end, resource_size(res2));
	printk("IRQ Number of Device :%d\n", platform_get_irq(pdev, 0));

	gpio_number = *(int *)(pdev->dev.platform_data);
	printk("GPIO Number is %d\n", gpio_number);
	return 0; 
}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,11,0))
static int sample_drv_remove(struct platform_device *pdev)
{
	return 0;
}
#else
static void sample_drv_remove(struct platform_device *pdev)
{
}
#endif

static struct platform_driver sample_pldriver = 
{
	.probe          = sample_drv_probe,
	.remove         = sample_drv_remove,
	.driver = 
	{
		.name  = DRIVER_NAME,
	},
};

static __init int init_platform_drv(void)
{
	printk("Welcome to sample Platform driver.... \n");

	/* Registering with Kernel */
	platform_driver_register(&sample_pldriver);

	return 0;
}

static void __exit exit_platform_drv(void)
{
	printk("Exiting sample Platform driver... \n");

	/* Unregistering from Kernel */
	platform_driver_unregister(&sample_pldriver);

	return;
}

module_init(init_platform_drv);
module_exit(exit_platform_drv);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Sample Platform Driver");
MODULE_LICENSE("GPL");
