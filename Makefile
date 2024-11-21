# Original author : Adriano Marto Reis
# Original sourc  : https://github.com/adrianomarto/soft_uart
# Modified by     : Hippy
            
obj-m += soft_uart.o

soft_uart-objs := module.o raspberry_soft_uart.o queue.o

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

release:
	make clean
	tar --exclude='.git' -zcvf /home/pi/soft_uart_hippy.tar.gz .
