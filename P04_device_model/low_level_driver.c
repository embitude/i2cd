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
#include "i2c_char.h"

/*
 * Assignment for the device model
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

int i2c_transmit(struct i2c_msg *msg, size_t count)
{
	/* Same as the previous assignment */
	return 0;
}

int i2c_receive(struct i2c_msg *msg, size_t count)
{	
	/* Same as the previous assignment */
	return 0;
}

static void omap_i2c_set_speed(struct omap_i2c_dev *dev)
{
	/* Same as the previous assignment */
}

int omap_i2c_init(struct omap_i2c_dev *dev)
{
	/* Same as the previous assignment */
	return 0;
}

static int sample_drv_probe(struct platform_device *pdev) 
{
	struct resource *res = NULL;
	/*
	 * TODO: Get the physical address from the platform device and 
	 * map it to the virtual address using ioremap.
	 * Accordingly, update the 'base' field of omap_i2c_dev
	 */
	printk("Memory Area1\n");
	printk("Start:%lx, End:%lx Size:%d\n", (unsigned long)res->start, 
			(unsigned long)res->end, resource_size(res));
	printk("Resource size = %x\n", resource_size(res));

	/*
	 * TODO: Get the clock_frequency from the platform_data of
	 * plaform device and assign it to speed field omap_i2c_dev
	 */
	printk("Clock frequency is %u\n", i2c_dev.speed);
	/* Copy rest of the stuff from previous assignment */

	return 0;
}

static int sample_drv_remove(struct platform_device *pdev)
{
	/* Copy the exit module from previous assignment */
	return 0;
}

//TODO: Populate the platform driver structure
static struct platform_driver sample_pldriver = 
{
};

static __init int omap_i2c_init_driver(void)
{
	//TODO: Register the platform driver
	return 0;
}

static void __exit omap_i2c_exit_driver(void)
{
	//TODO: Un-register the platform driver
}
module_init(omap_i2c_init_driver);
module_exit(omap_i2c_exit_driver);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Low level I2C driver");
MODULE_LICENSE("GPL");
