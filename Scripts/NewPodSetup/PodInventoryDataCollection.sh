#!/bin/bash

#!!! Copilot created, works MOST of the time

# Get the serial number of the nano from lshw
NANOSERIAL=$(lshw -class system | grep serial | awk '{print $2}')

# Get the MAC of the nano from lshw but only return the first line
NANOMAC=$(lshw -class network | grep serial | awk '{print $2}' | head -n 1)

# Get the Kinect serial number from AzureKinectFirmwareTool
KINECTSERIAL=$(AzureKinectFirmwareTool -q | grep Serial | awk '{print $4}')

# Get the capacity of the USB drive mounted at /mnt/USB from lsblk
mount /mnt/USB
USBCAPACITY=$(lsblk -o NAME,SIZE,MOUNTPOINT | grep /mnt/USB | awk '{print $2}')
umount /mnt/USB

# run lshw, and return only the storage class, grab the vendor line
USBVENDOR=$(sudo lshw -class storage | grep vendor | cut -d':' -f2-)
USBPRODUCT=$(sudo lshw -class storage | grep product | cut -d':' -f2-)
USBSERIAL=$(sudo lshw -class storage | grep serial | cut -d':' -f2-)

# Print these four variables on the same line in the order Kinect Serial, Nano Serial, Nano Mac, USB Capacity
echo $KINECTSERIAL $NANOSERIAL $NANOMAC $USBCAPACITY $USBVENDOR $USBPRODUCT
