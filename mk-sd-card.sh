SD_KERNEL_PARTITION=/dev/mmcblk0p2
NEW_DRIVERS_DIR=./drivers


#dd if=./arch/arm/boot/zImage of=/dev/sdf2

dd if=./arch/arm/boot/zImage of=$SD_KERNEL_PARTITION
cp -f $NEW_DRIVERS_DIR/usb/host/ehci-hcd.ko /media/ba76f865-871f-43b1-a065-a03616b0d158
cp -f $NEW_DRIVERS_DIR/ata/sata_mv.ko /media/ba76f865-871f-43b1-a065-a03616b0d158
cp -f $NEW_DRIVERS_DIR/mtd/nand/ts7800.ko /media/ba76f865-871f-43b1-a065-a03616b0d158
cp -f $NEW_DRIVERS_DIR/mmc/host/tssdcard.ko /media/ba76f865-871f-43b1-a065-a03616b0d158
cp -f $NEW_DRIVERS_DIR/serial/tsuart1.ko /media/ba76f865-871f-43b1-a065-a03616b0d158
cp -f $NEW_DRIVERS_DIR/serial/tsuart7800.ko /media/ba76f865-871f-43b1-a065-a03616b0d158
cp -f $NEW_DRIVERS_DIR/usb/core/usbcore.ko /media/ba76f865-871f-43b1-a065-a03616b0d158
ls -l /media/ba76f865-871f-43b1-a065-a03616b0d158/*.ko
rm -rf /media/1b5fcc3a-4e7b-403e-ab78-a4c1c41a88de/lib/modules/2.6.34
cp -r newmodules/lib/ /media/1b5fcc3a-4e7b-403e-ab78-a4c1c41a88de/
sync
sync
