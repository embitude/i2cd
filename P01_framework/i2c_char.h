#ifndef I2C_CHAR_H
#define I2C_CHAR_H

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>

/* timeout waiting for the controller to respond */
#define OMAP_I2C_TIMEOUT (msecs_to_jiffies(1000))
#define ENTER() printk("\n###### In %s ######\n", __func__);

struct omap_i2c_dev {
	dev_t devt;
	struct cdev cdev;
	struct class *i2c_class;
};


/* Test Functions */
int i2c_transmit(struct i2c_msg *i2c_msg, size_t len);
int i2c_receive(struct i2c_msg *i2c_msg, size_t len);

/* Char Driver Interface */
int chrdrv_init(struct omap_i2c_dev *i2c_dev);
void chrdrv_exit(struct omap_i2c_dev *i2c_dev);

#endif
