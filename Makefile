# Original author : Adriano Marto Reis
# Original sourc  : https://github.com/adrianomarto/soft_uart
# Modified by     : Hippy
            
obj-m += soft_uart.o

soft_uart-objs := module.o raspberry_soft_uart.o queue.o

ccflags-y := -Wno-incompatible-pointer-types

RELEASE = $(shell uname -r)
LINUX = /usr/src/linux-headers-$(RELEASE)

all:
	$(MAKE) -C $(LINUX) M=$(PWD) modules

clean:
	$(MAKE) -C $(LINUX) M=$(PWD) clean

install:
	sudo install -m 644 -c soft_uart.ko /lib/modules/$(RELEASE)
	sudo depmod

insmod:
	sudo insmod soft_uart.ko

rmmod:
	sudo rmmod soft_uart.ko

modinfo:
	modinfo soft_uart

