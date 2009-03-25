#!/bin/bash

# Mount a local .img qemi file on the local host via the new qemu-nbd
# The syntax specify the file, the mount point, and the partition

# example:
# sudo ./mini2440/qemu-local-img-mount.sh mini2440/emdebian_1GB.img /mnt/arm p2
#
# Mounting mini2440/emdebian_1GB.img - /dev/mapper/nbd1p2 to /mnt/arm
# /dev/mapper/nbd1p2      903024    225196    631848  27% /mnt/arm
# ...
# umount /mnt/arm
# ./qemu-nbd -d /dev/nbd1
#

if [ ! -f $1 -o ! -d $2 ]; then
	echo "$0 <image filename> <mount point> [pX]"
	exit 1
fi

nbd=nbd1

modprobe nbd

./qemu-nbd -c /dev/$nbd $1
kpartx -a /dev/$nbd

if [ -b "/dev/mapper/${nbd}$3" ]; then
	part="/dev/mapper/${nbd}$3" 
else
	part="/dev/mapper/${nbd}p1" 
fi

echo Mounting $1 - $part to $2

#ls -l /dev/mapper/${nbd}*

mount $part $2

df -h|grep $part
