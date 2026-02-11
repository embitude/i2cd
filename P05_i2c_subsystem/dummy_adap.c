#include <linux/module.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>


static u32 dummy_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK) |
	       I2C_FUNC_PROTOCOL_MANGLING;
}

int dummy_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	printk("Dummy transfer invoked\n");
	/*
	 * TODO 7.18: Print the i2c_msg buff contents for tx and 
	 * update the contents with some data for receive
	 */
	return 0;
}

//TODO 7.14: Populate the i2c_algo with callback functions
static const struct i2c_algorithm dummy_i2c_algo = {
};

static int dummy_i2c_probe(struct platform_device *pdev)
{
	struct i2c_adapter	*adap;
	int r;

	printk("Dummy Adapter probe invoked\n");
	adap = devm_kzalloc(&pdev->dev, sizeof(struct i2c_adapter), GFP_KERNEL);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strscpy(adap->name, "Dummy I2C adapter", sizeof(adap->name));

	//TODO 7.15: Initialize the adapter algo field

	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	/* i2c device drivers may be active on return from add_adapter() */
	adap->nr = pdev->id;

	// TODO 7.16: Register the adapter with i2c subsystem
	if (r) {
		printk("Failure adding adapter\n");
		return r;
	}
	
	platform_set_drvdata(pdev, adap);
	return r;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,11,0))
static int dummy_i2c_remove(struct platform_device *pdev)
#else
static void dummy_i2c_remove(struct platform_device *pdev)
#endif
{
	struct i2c_adapter *adapter = platform_get_drvdata(pdev);
	//TODO 7.17: De-register the adapter from the I2C subystem

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,11,0))
	return 0;
#endif
}

//TODO 7.12: Populate the dummy_i2c_of_match with compatible property
static const struct of_device_id dummy_i2c_of_match[] = {
	{
	},
	{ }, //Don't delete this
};
MODULE_DEVICE_TABLE(of, dummy_i2c_of_match);

//TODO 7.13: Populate the dummy_i2c_driver
//Use .of_match_table = of_match_ptr for matching
static struct platform_driver dummy_i2c_driver = {
};

static int __init dummy_i2c_init_driver(void)
{
	return platform_driver_register(&dummy_i2c_driver);
}

static void __exit dummy_i2c_exit_driver(void)
{
	platform_driver_unregister(&dummy_i2c_driver);
}
module_init(dummy_i2c_init_driver);
module_exit(dummy_i2c_exit_driver);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Dummy I2C bus adapter");
MODULE_LICENSE("GPL");
