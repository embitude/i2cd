# If called directly from the command line, invoke the kernel build system.
ifeq ($(KERNELRELEASE),)

	KERNEL_SOURCE := <Kernel source path>
	PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNEL_SOURCE) SUBDIRS=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SOURCE) SUBDIRS=$(PWD) clean

# Otherwise KERNELRELEASE is defined; we've been invoked from the
# kernel build system and can use its language.
else

	obj-m := i2c.o platform_device.o platform_driver.o low_level_device.o i2c_dtb.o
	i2c-y := i2c_char.o low_level_driver.o
	i2c_dtb-y := i2c_char.o low_level_driver_dtb.o

endif
