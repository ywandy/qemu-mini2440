#!/bin/bash

base=$(dirname $0)

echo Starting in $base

name_nand="$base/mini2440_nand.bin"
name_sd="$base/mini2440_sd.img"

if [ ! -f "$name_nand" ]; then
	echo $0 : creating NAND empty image : "$name_nand"
	dd if=/dev/zero of="$name_nand" bs=528 count=131072
fi
#if [ -f "$name_sd" ]; then
#	optional="$optional -sd $name_sd"
#	optional=""
	# use a real SD card
	optional="$optional -sd /dev/sdd"
#fi
# remove old socket
rm -rf .mini2440_monitor

cmd="$base/../arm-softmmu/qemu-system-arm \
	-M mini2440 -m 66 \
	-semihosting \
	-serial stdio \
	-kernel /tftpboot/uImage \
	-mtdblock "$name_nand" \
	$optional \
	-show-cursor -usb -usbdevice keyboard \
	-net nic,vlan=0 \
	-net tap,vlan=0,ifname=tap0 \
	-monitor unix:.mini2440_monitor,server,nowait"

echo $cmd
$cmd
