We use the ts supplied 2.6.34 kernel

Kernel config:
       enable ppp_async
       disable General setup->Automatically append version information ...
       preempt_rt (already enabled)


Instructions:
	Patch:
		Fix/patch hardware clock bug (https://lkml.org/lkml/2011/3/4/223)
		Fix sd card issue (https://groups.yahoo.com/neo/groups/ts-7000/conversations/topics/22375)
	Build:
		http://forum.embeddedarm.com/showthread.php?117-2-6-34-kernel-upgrage-on-TS-7800


Instruction Summary:
    "tar xvzf ts7800-crosstool-linux-gnueabi-2008q3-2.tar.gz" to extract the cross compiler in the current directory.
    "tar xvzf 2.6.34.tar.gz" to extract the source code in the current directory.
    "cd linux-2.6.34"
    "export CROSS_COMPILE=../arm-2008q3/bin/arm-none-eabi-"
    "export ARCH=arm"
    "make ts7800_defconfig"
    "make menuconfig"    #add kernel config changes
    "make && make modules"
    "mkdir newmodules"
    "INSTALL_MOD_PATH=newmodules_hwclock make modules_install"
    "sh ../mk-sd-card.sh.txt"