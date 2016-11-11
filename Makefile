ZJH_DEBUG = y

ifeq ($(ZJH_DEBUG),y)
ccflags-y += -D_ZJH_DBG_ON_
endif

ifneq ($(KERNELRELEASE),)
	obj-m		:=	mpu_module.o
	mpu_module-objs	:=	mpu.o fifo.o calc_angle.o
else
	KERNELDIR	?=	~/code/rpi-linux/linux-rpi-3.2.27
	PWD		:=	$(shell pwd)

default:
	$(MAKE)	-C	$(KERNELDIR)		M=$(PWD)	KBUILD_EXTRA_SYMBOL=./Module.symvers  \
	ARCH=arm  CROSS_COMPILE=arm-linux-gnueabihf- -$(DEBUG_FLAG) modules

clean:
	rm  -f  *.ko  *.o  *.mod.o *.mod.c  *.order   *~  *.symvers *.cmd

endif
	
	
