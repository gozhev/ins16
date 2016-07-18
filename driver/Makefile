TARGET := fast-mpu6050.ko
SRCS := fast_mpu6050.c

COMPILE := CROSS

REMOTE_HST := 192.168.0.3
REMOTE_USR := root
REMOTE_DIR := /lib/modules/4.4.14+/extra

MODULE_BASENAME := $(TARGET:%.ko=%)
obj-m += $(MODULE_BASENAME).o
$(MODULE_BASENAME)-objs := $(SRCS:%.c=%.o)

ifeq ($(COMPILE),CROSS)

KDIR := /home/student/dvlp/raspberrypi/linux
PARAM := ARCH=arm CROSS_COMPILE=/home/student/dvlp/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-

else

KDIR := /lib/modules/$(shell uname -r)/build
PARAM := 

endif

all: modules

modules:
	make $(PARAM) -C $(KDIR) M=$(PWD) modules

clean:
	make $(PARAM) -C $(KDIR) M=$(PWD) clean

upload:
	scp $(TARGET) $(REMOTE_USR)@$(REMOTE_HST):$(REMOTE_DIR)
