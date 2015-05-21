#!/bin/bash

STAGING_DIR=../openwrt_stick/OpenWrt-SDK-ramips-rt305x_gcc-4.8-linaro_uClibc-0.9.33.2.Linux-x86_64/staging_dir/   ../openwrt_stick/OpenWrt-SDK-ramips-rt305x_gcc-4.8-linaro_uClibc-0.9.33.2.Linux-x86_64/staging_dir/toolchain-mipsel_24kec+dsp_gcc-4.8-linaro_uClibc-0.9.33.2/bin/mipsel-openwrt-linux-gcc rtlsdr_test.c -I../openwrt_stick/OpenWrt-SDK-ramips-rt305x_gcc-4.8-linaro_uClibc-0.9.33.2.Linux-x86_64/staging_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/usr/include -o rtlsdr_test.openwrt
