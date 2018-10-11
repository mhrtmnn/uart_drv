-include cfg.local

ARCHITECTURE := arm
TOOLCHAIN := $(MY_TOOLCHAIN)
COMPILER := $(MY_COMPILER)

# MY_REPO_DIR needs to be exported
VERSION_TAG := \"$(shell git --git-dir=$(MY_REPO_DIR) rev-parse HEAD)\"
EXTRA_CFLAGS += -D MOD_VER=$(VERSION_TAG) -Wall
PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) ARCH=$(ARCHITECTURE) CROSS_COMPILE=$(TOOLCHAIN) CC=$(COMPILER) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

deploy: modules
	scp uart_drv.ko root@10.10.0.40:/root

.PHONY: modules clean

obj-m += uart_drv.o

#"make print-XXX" prints value of variable XXX
print-% : ; @echo $* = $($*)
