obj-m += gpio_comm_drv.o

CFLAGS_gpio_comm_drv.o := -DDEBUG

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean