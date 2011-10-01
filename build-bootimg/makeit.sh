#!/bin/sh
cd ramdisk
find . | cpio -o -H newc | gzip > ../ramdisk.gz
cd ..
./mkbootimg --cmdline 'semcandroidboot.serialno=CB511SW0YT semcandroidboot.startup=0x0000001A semcandroidboot.000008A2=534F2D303142' --kernel zImage --ramdisk ramdisk.gz --base 0x20000000 -o boot.img

cd ramdisk.froyo
find . | cpio -o -H newc | gzip > ../ramdisk.froyo.gz
cd ..
./mkbootimg --cmdline 'semcandroidboot.serialno=CB511SW0YT semcandroidboot.startup=0x0000001A semcandroidboot.000008A2=534F2D303142' --kernel zImage --ramdisk ramdisk.froyo.gz --base 0x20000000 -o boot.froyo.img

cd ramdisk.cm70
find . | cpio -o -H newc | gzip > ../ramdisk.cm70.gz
cd ..
./mkbootimg --cmdline 'semcandroidboot.serialno=CB511SW0YT semcandroidboot.startup=0x0000001A semcandroidboot.000008A2=534F2D303142' --kernel zImage --ramdisk ramdisk.cm70.gz --base 0x20000000 -o boot.cm70.img

cd ramdisk.cm71
find . | cpio -o -H newc | gzip > ../ramdisk.cm71.gz
cd ..
./mkbootimg --cmdline 'semcandroidboot.serialno=CB511SW0YT semcandroidboot.startup=0x0000001A semcandroidboot.000008A2=534F2D303142' --kernel zImage --ramdisk ramdisk.cm71.gz --base 0x20000000 -o boot.cm71.img

cd ramdisk.gb
find . | cpio -o -H newc | gzip > ../ramdisk.gb.gz
cd ..
#./mkbootimg --cmdline 'semcandroidboot.serialno=CB511SW0YT semcandroidboot.startup=0x0000001A semcandroidboot.000008A2=534F2D303142' --kernel zImage --ramdisk ramdisk.gb.gz --base 0x20000000 -o boot.gb.img
./mkbootimg --cmdline 'androidboot.hardware=es209ra vmalloc=280M g_android.product_id=0x312E console=ttyMSM0 semcandroidboot.serialno=CB511SW0YT semcandroidboot.startup=0x0000001A semcandroidboot.000008A2=583130' --kernel zImage --ramdisk ramdisk.gb.gz --base 0x20000000 -o boot.gb.img

#./mkbootimg --cmdline 'mtdparts=msm_nand:0x00440000@0x3fbc0000(appslog),0x06f40000@0x38c80000(cache),0x160a0000@0x05ae0000(system),0x1d100000@0x1bb80000(userdata) androidboot.hardware=es209ra vmalloc=128M g_android.product_id=0x312E console=ttyMSM0 semcandroidboot.serialno=CB511SW0YT semcandroidboot.startup=0x0000001A semcandroidboot.000008A2=534F2D303142' --kernel zImage --ramdisk ramdisk.gz --base 0x20000000 -o boot.img
