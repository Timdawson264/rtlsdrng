#!/bin/bash

OPENWRT=$HOME/src/openwrt/bin/ramips/OpenWrt-SDK-ramips-rt305x_gcc-4.8-linaro_uClibc-0.9.33.2.Linux-x86_64

KERNEL=$OPENWRT/build_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/linux-ramips_rt305x/linux-3.18.11

STAGING_DIR=$OPENWRT/staging_dir

TOOLS=$STAGING_DIR/toolchain-mipsel_24kec+dsp_gcc-4.8-linaro_uClibc-0.9.33.2/

INC=$STAGING_DIR/target-mipsel_24kec+dsp_uClibc-0.9.33.2/usr/include

LD=$TOOLS/bin/mipsel-openwrt-linux-uclibc-ld
CC=$TOOLS/bin/mipsel-openwrt-linux-uclibc-gcc
CROSS_COMPILE="mips-openwrt-linux-uclibc-"


STAGING_DIR=$STAGING_DIR $CC rtlsdr_test.c -I $INC -o rtlsdr_test.openwrt
