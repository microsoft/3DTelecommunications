#!/bin/bash
#!!! Co-pilot created, works MOST of the time
#BEFORE USE verify that the USB drive is at sdb1 or sda1 and update the UUID= line accordingly

## Get the UUId of sdb1 from /dev/disk/by-uuid
UUID=$(ls -l /dev/disk/by-uuid | grep sdb1 | awk '{print $9}')

## Remove the line in fstab that mounts a ramdisk to /mnt/RamDisk
sed -i "/.*\/mnt\/RamDisk.*/d" /etc/fstab

## Replace the line in /etc/fstab that contains /mnt/USB with a new line that mounts
## /dev/disk/by-uuid/$UUID  to /mnt/USB with the correct options
sed -i "s/.*\/mnt\/USB.*/\/dev\/disk\/by-uuid\/$UUID  \/mnt\/USB        ext4    user,noauto,nofail,rw,exec      0       0/" /etc/fstab

