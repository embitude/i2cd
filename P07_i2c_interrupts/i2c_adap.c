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
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include "i2c_adap.h"

/*
 * Assignment for the interrupt enabled driver
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
static void omap_i2c_resize_fifo(u8 size, bool is_rx) 
{ 
	u16	buf; 
 
	/* 
	 * Set up notification threshold based on message size. We're doing 
	 * this to try and avoid draining feature as much as possible. Whenever 
	 * we have big messages to transfer (bigger than our total fifo size) 
	 * then we might use draining feature to transfer the remaining bytes. 
	 */ 
 
	i2c_dev.threshold = clamp(size, (u8) 1, i2c_dev.fifo_size); 
 
	buf = omap_i2c_read_reg(&i2c_dev, OMAP_I2C_BUF_REG); 
 
	if (is_rx) { 
		/* Clear RX Threshold */ 
		buf &= ~(0x3f << 8); 
		buf |= ((i2c_dev.threshold - 1) << 8) | OMAP_I2C_BUF_RXFIF_CLR; 
	} else { 
		/* Clear TX Threshold */ 
		buf &= ~0x3f; 
		buf |= (i2c_dev.threshold - 1) | OMAP_I2C_BUF_TXFIF_CLR; 
	} 
	omap_i2c_write_reg(&i2c_dev, OMAP_I2C_BUF_REG, buf); 
}

int i2c_txrx(struct i2c_adapter *adap, struct i2c_msg msg[], int count)
{
	u16 w, j = 0;
	int i2c_error = 0;
	unsigned long timeout;

	ENTER();

	while (j < count) {
			i2c_dev.receiver = !!(msg[j].flags & I2C_M_RD);
			omap_i2c_resize_fifo(msg[j].len, i2c_dev.receiver);
			// TODO: Update buff to point to i2c_msg buf
			
			i2c_error = count;
			/* 
			 * TODO: Update CNT, and SA registers as per the
			 * i2c_msg.
			 */
			/* Clear the FIFO Buffers */
			w = omap_i2c_read_reg(&i2c_dev, OMAP_I2C_BUF_REG);
			w |= OMAP_I2C_BUF_RXFIF_CLR | OMAP_I2C_BUF_TXFIF_CLR;
			omap_i2c_write_reg(&i2c_dev, OMAP_I2C_BUF_REG, w);

			printk("##### Sending/Receiving %d byte(s) on the I2C bus ####\n", msg[j].len);

			// TODO: Update the CON register. Set the TRX bit only if transmitting
			reinit_completion(&i2c_dev.cmd_complete);
			i2c_dev.cmd_err = 0;
			printk("Wating...\n");
			/* 
			 * TODO: Wait for completion with timeout OMAP_I2C_TIMEOUT. 
			 * Refer include/linux/completion.h
			 */
			if (timeout == 0) {
					printk("controller timed out\n");
					return -ETIMEDOUT;
			}
			flush_fifo(&i2c_dev);
			omap_i2c_write_reg(&i2c_dev, OMAP_I2C_STAT_REG, 0XFFFF);
			j++;
	}
	return i2c_error;
}

static u32 i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK) |
	       I2C_FUNC_PROTOCOL_MANGLING;
}

// TODO: Populate the i2c_algo structure
static const struct i2c_algorithm i2c_algo = {
	.master_xfer	= i2c_txrx,
	.functionality	= i2c_func,
};

static void omap_i2c_set_speed(struct omap_i2c_dev *dev)
{
		/* Same as previous assignment */
}

int omap_i2c_init(struct omap_i2c_dev *dev)
{
	omap_i2c_set_speed(dev);
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);

	/* Take the I2C module out of reset: */
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);
	
	// TODO: Update the 'iestate' field with desired events such as XRDY. Make sure to include XDR and RDR

	// TODO: Update the OMAP_I2C_IE_REG register

	flush_fifo(dev);
	omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, 0XFFFF);
	omap_i2c_wait_for_bb(dev);

	return 0;
}

static void receive_data(struct omap_i2c_dev *omap, u8 num_bytes)
{
        u8 w;
		printk("Recieving %d bytes of data\n", num_bytes);
        while (num_bytes--) {
                w = omap_i2c_read_reg(omap, OMAP_I2C_DATA_REG);
                *omap->buf++ = w;
                omap->buf_len--;
       }
}

static int transmit_data(struct omap_i2c_dev *omap, u8 num_bytes)
{
	u8 w;
	printk("Transmitting %d bytes of data\n", num_bytes);
	while (num_bytes--) {
		w = *omap->buf++;
		omap->buf_len--;
		omap_i2c_write_reg(omap, OMAP_I2C_DATA_REG, w);
	}
	return 0;
}

static irqreturn_t thread_fn(int this_irq, void *dev_id) {
	u16 bits, status;
	int err = 0, ret = 0;

	do {
		bits = omap_i2c_read_reg(&i2c_dev, OMAP_I2C_IE_REG);
		status = omap_i2c_read_reg(&i2c_dev, OMAP_I2C_STAT_REG);
		status &= bits;
		
		/* If we're in receiver mode, ignore XDR/XRDY */
		if (i2c_dev.receiver)
			status &= ~(OMAP_I2C_STAT_XDR | OMAP_I2C_STAT_XRDY);
		else
			status &= ~(OMAP_I2C_STAT_RDR | OMAP_I2C_STAT_RRDY);
        if (!status) {
                goto out;
        }

		if (status & OMAP_I2C_STAT_XRDY) {
			printk("Got XRDY\n");
			// TODO: Transmit the threshold size bytes & clear the XRDY
			continue;   
		}
		if (status & OMAP_I2C_STAT_RRDY) {
			printk("Got RRDY\n");
			// TODO: Receive the threshold sized bytes & clear the RRDY 
			continue;
        }
		if (status & OMAP_I2C_STAT_XDR) {
			printk("Got XDR\n");
			// TODO: Transmit the remaining bytes & clear the XDR
			continue;
		}
		if (status & OMAP_I2C_STAT_RDR) {
			printk("Got RDR\n");
			// TODO: received the remaining bytes  & clear the RDR
			continue;
		}

		if (status & OMAP_I2C_STAT_ARDY) {	
			printk("Got ARDY\n");
			omap_i2c_ack_stat(&i2c_dev, OMAP_I2C_STAT_ARDY);   
			break;
		}
	} while (status);

	i2c_dev.cmd_err |= err;
	// TODO: Signal the completion 
	complete(&(i2c_dev.cmd_complete));

out:
	return IRQ_HANDLED;
}

static irqreturn_t irq_fn(int irq, void *dev_id)
{
	struct omap_i2c_dev *omap = dev_id;
	irqreturn_t ret = IRQ_HANDLED;
	u16 mask, stat;

	stat = omap_i2c_read_reg(omap, OMAP_I2C_STAT_REG);
	mask = omap_i2c_read_reg(omap, OMAP_I2C_IE_REG);

	if (stat & mask)
		ret = IRQ_WAKE_THREAD;
	return ret;
}

static int sample_drv_probe(struct platform_device *pdev) 
{
	struct resource *res = NULL;
	struct device_node *np = pdev->dev.of_node;
	u32 clk_frq;
	struct i2c_adapter	*adap;
	int r;
	u16 s;

	// TODO: Get the irq number
	printk("Irq number is %d\n", i2c_dev.irq);

	// TODO: Initialize the completion data structure
	init_completion(&(i2c_dev.cmd_complete));
	/*
	 * TODO: Get the physical address from the pdev and 
	 * map it to the virtual address using ioremap.
	 * Accordingly, update the 'base' field of omap_i2c_dev
	 */

	if (!res) 
	{
		pr_err(" Specified Resource Not Available... 1\n");
		return -1;
	}
	printk(KERN_ALERT "\n Memory Area1\n");
	printk(KERN_ALERT "Start:%lx, End:%lx Size:%d\n", (unsigned long)res->start, 
			(unsigned long)res->end, resource_size(res));
	printk("Resource size = %x\n", resource_size(res));

	i2c_dev.regs = (u8 *)reg_map_ip_v2;

	/*
	 * TODO: Get the the total fifo depth from OMAP_I2C_BUFTAT_REG
	 * and accordingly update fifo_size field in i2c_dev to half of
	 * the total fifo depth. This will serve as notification threshold
	 */
	/*
	 * TODO: Get the clock_frequency from the dtb node
	 * into clk_freq
	 */
	printk("Clock frequency is %u\n", i2c_dev.speed);
	omap_i2c_init(&i2c_dev);

	/* 
	 * TODO: Register the threaded irq interrupt with irq_fn as top half
	 * and irq bottom half. Use the flag IRQF_ONESHOT
	 */
	

	// TODO: Perform adapter related configuration as per earlier assignments
	platform_set_drvdata(pdev, adap);
	return 0;
}

static int sample_drv_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(pdev);
	// TODO: De-register the adapter from the I2C subystem

	// TODO: Free up the irq number

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
MODULE_DESCRIPTION("Low level I2C driver with interrupts");
MODULE_LICENSE("GPL");
