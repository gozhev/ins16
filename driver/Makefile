#BOARD_KDIR =
#BOARD_PARAM = ARCH=arm CROSS_COMPILE=arm-linux-gueabihf-

KDIR_SELF = /lib/modules/$(shell uname -r)/build
ARG_SELF = 

KDIR := $(KDIR_SELF)
PARAM := $(PARAM_SELF)

obj-m = kxcjk.o

all: modules

modules:
	make $(PARAM) -C $(KDIR) M=$(PWD) modules

clean:
	make $(PARAM) -C $(KDIR) M=$(PWD) clean

