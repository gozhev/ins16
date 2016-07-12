#Makefile for out of kernel builds 
# used for building driver module against existing kernel
# kpowell@kionix.com

DRIVERNAME = knxjif

ifneq ($(KERNELRELEASE),)
# call from kernel build system

obj-m += knxjif.o

else

KERNEL_PATH ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

modules:
	$(MAKE) -C $(KERNEL_PATH) M=$(PWD) modules

endif

#ccflags-y := -D$(KTARG) -I$(M)
ccflags-y := -I$(M)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

depend .depend dep:
	$(CC) $(CFLAGS) -M *.c > .depend


ifeq (.depend,$(wildcard .depend))
include .depend
endif
