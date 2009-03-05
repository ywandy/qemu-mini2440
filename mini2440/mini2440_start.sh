#!/bin/bash

base=$(dirname $0)

echo Starting in $base

name_nand="$base/mini2440_nand.bin"
name_snapshots="$base/mini2440_snapshots.img"

if [ ! -f "$name_nand" ]; then
	echo $0 : creating NAND empty image : "$name_nand"
	dd if=/dev/zero of="$name_nand" bs=528 count=131072
fi
if [ ! -f "$name_snapshots" ]; then
	echo $0 : creating empty snapshot image : "$name_snapshots"
	qemu-img create "$name_snapshots" 100MB
fi

# remove old socket
rm -rf .mini2440_monitor

cmd="$base/../arm-softmmu/qemu-system-arm \
	-M mini2440 $* \
	-drive file=$name_snapshots,snapshot=on \
	-serial stdio \
	-kernel /tftpboot/uImage \
	-mtdblock "$name_nand" \
	-show-cursor \
	-usb -usbdevice keyboard -usbdevice mouse \
	-net nic,vlan=0 \
	-net tap,vlan=0,ifname=tap0 \
	-monitor telnet::5555,server,nowait"

#	-monitor unix:.mini2440_monitor,server,nowait"

echo $cmd
$cmd
