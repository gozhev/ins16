TARGET := nav
CSRCS := nav.c engine.c

CFLAGS := -lm

REMOTE_HST := 192.168.0.3
REMOTE_USR := root
REMOTE_DIR := /root

ifeq ($(PLATFORM),board)

CROSS := /home/student/dvlp/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-

else

CROSS := 

endif

all: $(TARGET)

$(TARGET): $(CSRCS)
	$(CROSS)gcc $(CFLAGS) -o $(TARGET) $(CSRCS)

clean:
	rm -f $(TARGET)
	rm -f out.dat

upload: $(TARGET)
	scp $(TARGET) $(REMOTE_USR)@$(REMOTE_HST):$(REMOTE_DIR)

