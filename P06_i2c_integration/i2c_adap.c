#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/platform_data/i2c-omap.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include "i2c_adap.h"

/*
 * Assignment for the integrating low level driver with framework
 */

#define DRIVER_NAME "Sample_Pldrv"

static struct omap_i2c_dev i2c_dev;

inline void omap_i2c_write_reg(struct omap_i2c_dev *i2c_dev,
				      int reg, u16 val)
{
	__raw_writew(val, i2c_dev->base + i2c_dev->regs[reg]);
}

inline u16 omap_i2c_read_reg(struct omap_i2c_dev *i2c_dev, int reg)
{
	return __raw_readw(i2c_dev->base + i2c_dev->regs[reg]);
}

inline void omap_i2c_ack_stat(struct omap_i2c_dev *dev, u16 stat)
{
	omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);
}

/*
 * Waiting on Bus Busy
 */
int omap_i2c_wait_for_bb(struct omap_i2c_dev *dev)
{
	unsigned long timeout;

	timeout = jiffies + OMAP_I2C_TIMEOUT;
	while (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG) & OMAP_I2C_STAT_BB) {
		if (time_after(jiffies, timeout)) {
			printk("timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}

	return 0;
}

void flush_fifo(struct omap_i2c_dev *dev)
{
	unsigned long timeout;
	u32 status;

	timeout = jiffies + OMAP_I2C_TIMEOUT;
	while ((status = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG)) & OMAP_I2C_STAT_RRDY) {
		omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
		omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RRDY);
		if (time_after(jiffies, timeout)) {
			printk(KERN_ALERT "timeout waiting for bus ready\n");
			break;
		}
		msleep(1);
	}
}

u16 wait_for_event(struct omap_i2c_dev *dev)
{
	unsigned long timeout = jiffies + OMAP_I2C_TIMEOUT;
	u16 status;

	while (!((status = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG)) & 
				(OMAP_I2C_STAT_ROVR | OMAP_I2C_STAT_XUDF | 
				 OMAP_I2C_STAT_XRDY | OMAP_I2C_STAT_RRDY | 
				 OMAP_I2C_STAT_ARDY | OMAP_I2C_STAT_NACK | 
				 OMAP_I2C_STAT_AL))) {
		if (time_after(jiffies, timeout)) {
			printk("time-out waiting for event\n");
			omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, 0XFFFF);
			return 0;
		}
		mdelay(1);
	}
	return status;
}

int i2c_txrx(struct i2c_adapter *adap, struct i2c_msg msg[], int count)
{
	int i2c_error = 0;
	/* Same as previous assignment */
	return i2c_error;
}

static u32 i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK) |
	       I2C_FUNC_PROTOCOL_MANGLING;
}

// TODO: Populate the i2c_also structure
static const struct i2c_algorithm i2c_algo = {
};

static void omap_i2c_set_speed(struct omap_i2c_dev *dev)
{
	/* Same as previous assignment */
}

int omap_i2c_init(struct omap_i2c_dev *dev)
{
	/* Same as previous assignment */	
	return 0;
}

static int sample_drv_probe(struct platform_device *pdev) 
{
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	u32 clk_frq;
	struct i2c_adapter	*adap;
	int r;
	// TODO: Perform the I2C controller specific initialization
	
	// TODO: I2C adapter related configuration as per dummy adapter
	return 0;
}

static int sample_drv_remove(struct platform_device *pdev)
{
	/* Same as earlier assignment */
	return 0;
}

//TODO: Initialize the compatible property
static const struct of_device_id i2c_drv_dt[] = {
	{ },
	{ }
};

/* 
 * TODO: Populate the platform driver structure as
 * per previous assignment. In addition to this,
 * iniitialize the of_match_table field
 */
static struct platform_driver sample_pldriver = 
{
};

static __init int omap_i2c_init_driver(void)
{
	platform_driver_register(&sample_pldriver);
	return 0;
}

static void __exit omap_i2c_exit_driver(void)
{
	platform_driver_unregister(&sample_pldriver);
}
module_init(omap_i2c_init_driver);
module_exit(omap_i2c_exit_driver);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Low level I2C driver");
MODULE_LICENSE("GPL");
