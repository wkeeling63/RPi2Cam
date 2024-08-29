obj-m += racecam_drv.o

all: module dt
	echo Building RaceCam device tree Overlay and kernel module
module:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules
dt: racecam.dts
	dtc -@ -I dts -O dtb -o racecam.dtbo racecam.dts
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean
	rm -rf racecam.dtbo
