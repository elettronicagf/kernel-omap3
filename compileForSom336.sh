#choose the right one for your toolchain
CROSS_COMPILE=arm-angstrom-linux-gnueabi-

make CROSS_COMPILE=$CROSS_COMPILE omap3egf_defconfig
make CROSS_COMPILE=$CROSS_COMPILE uImage

