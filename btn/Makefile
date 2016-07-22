TARGET := btn
CSRCS := btn.c 

CFLAGS := -lm

REMOTE_HST := 192.168.0.3
REMOTE_USR := root
REMOTE_DIR := /root/dvlp/nav

ifneq ($(PLATFORM),SELF)

CROSS := /home/student/dvlp/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-

else

CROSS := 

endif

all: $(TARGET)

$(TARGET): $(CSRCS)
	$(CROSS)gcc $(CFLAGS) -o $(TARGET) $(CSRCS)

clean:
	rm -f $(TARGET)

upload: $(TARGET)
	scp $(TARGET) $(REMOTE_USR)@$(REMOTE_HST):$(REMOTE_DIR)

