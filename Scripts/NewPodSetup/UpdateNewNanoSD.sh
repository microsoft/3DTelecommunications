#!/bin/bash
### Update Nano script.  This script would be called on a linux host where you have inserted the pod's SD card
### You need to specify the number of the pod, the starting IP of the pod network (e.g. 192.168.101.1)
### The IP of the control panel, the number of pods, the device name of the SD card (e.g. /dev/sdd1)
### the desired username (peabody)
### the directory where the 3DTM binaries are located (typically downloads folder)
### and the mount path where the script should mount the device to do the work

NanoNumber=${1?"Error.  Please enter the number of this Pod (1-8)"}
StartingIP=${2?"Error.  Please enter the start IP (the ip of the first POD (192.168.1.1)"}
ControlPanelIP=${3?"Error.  Please enter the IP of the control panel"}
NumberOfPods=${4:-10}
Device=${5:-"/dev/sdd1"}
username=$(whoami)
BinariesDir=${6:-"/home/$username/Downloads"}
mountpath="/media/$username/USB"
# Parse the final octet
IFS='.'
octets=($StartingIP)
if [ ${#octets[@]} -lt 4 ]; then
	echo "Error.  Invalid IP address. Number of octets = ${#octets[@]}"
	exit -1
fi
finalOctet=${octets[3]}
# Make sure we have enough room for all of the Nanos without incrementing the second octet
max=$(($finalOctet + 8))
if [ $max -gt 255 ]; then
	echo "Error.  Can't allocate enough IPs on the final octet. Try starting with a lower final octet."
	exit -1
fi

ip=$((${octets[3]} + $NanoNumber - 1))
thisIP="${octets[0]}.${octets[1]}.${octets[2]}.$ip"

# Mount the SD card to $mountpath
mkdir -p $mountpath
sudo umount $Device || /bin/true
echo "Mounting the SD Card to $mountpath"
sudo mount $Device $mountpath
# Create a new etc/hosts
echo "Creating static routing table"
sudo chmod a+rwx $mountpath/etc
sudo rm $mountpath/etc/hosts
echo "127.0.0.1	localhost" > $mountpath/etc/hosts
echo "127.0.0.1	POD-${NanoNumber}" >> $mountpath/etc/hosts
echo "${ControlPanelIP}	ControlPanel" >> $mountpath/etc/hosts
START=1
END=$NumberOfPods+1
for (( i=$START; i<$END; i++))
do
	if [ $i -eq $NanoNumber ]; then 
		continue 
	fi
	nextIP=$((${octets[3]} + $i - 1))  
	echo "${octets[0]}.${octets[1]}.${octets[2]}.$nextIP	POD-${i}" >> $mountpath/etc/hosts
done
sudo chown root:root $mountpath/etc/hosts
sudo chmod 644 $mountpath/etc/hosts
# Create a new etc/hostname
echo "Setting hostname to POD-${NanoNumber}"
sudo chmod 777 $mountpath/etc/hostname
echo "POD-${NanoNumber}" > $mountpath/etc/hostname
sudo chmod 644 $mountpath/etc/hostname
sudo chown root:root $mountpath/etc/hostname
sudo chmod 644 $mountpath/etc/hostname

echo "Setting static IP network configuration.  IP is set to ${thisIP}"
sudo chmod 777 $mountpath/etc/network/
sudo chmod 777 $mountpath/etc/network/interfaces.d/
sudo rm $mountpath/etc/network/interfaces
echo "source-directory /etc/network/interfaces.d" > $mountpath/etc/network/interfaces
echo "source interfaces.d/eth0" >> $mountpath/etc/network/interfaces
sudo chmod 644 $mountpath/etc/network/interfaces
echo "auto eth0" > $mountpath/etc/network/interfaces.d/eth0
echo "iface eth0 inet static" >> $mountpath/etc/network/interfaces.d/eth0
echo "address ${thisIP}" >> $mountpath/etc/network/interfaces.d/eth0
echo "netmask 255.255.255.0" >> $mountpath/etc/network/interfaces.d/eth0
#echo "network ${octets[0]}.${octets[1]}.${octets[2]}.0"  >> $mountpath/etc/network/interfaces.d/eth0
#echo "broadcast ${octets[0]}.${octets[1]}.${octets[2]}.255"  >> $mountpath/etc/network/interfaces.d/eth0
echo "gateway ${octets[0]}.${octets[1]}.${octets[2]}.250" >> $mountpath/etc/network/interfaces.d/eth0
sudo chmod 644 $mountpath/etc/network/interfaces.d/eth0

echo "Setting up NTP configuration"
# Copy over the correct ntp.conf
if [ $NanoNumber -eq 1 ]; then
	sudo cp ./root/etc/ntp.nano000.conf $mountpath/etc/ntp.conf
else
	sudo cp ./root/etc/ntp.nanox.conf $mountpath/etc/ntp.conf
fi

echo "Installing logrotate configuration."
# Copy the new logrotate.d configuration
sudo cp ./root/etc/logrotate.d/K4AToFusion $mountpath/etc/logrotate.d/

echo "Installing unattended-upgrades configuration"
# Copy the unattended-upgrades config
sudo cp ./root/etc/apt/apt.conf.d/50unattended-upgrades $mountpath/etc/apt/apt.conf.d/

echo "Installing rc.local"
sudo cp ./root/etc/rc.local $mountpath/etc/
echo "Installing new fstab"
sudo cp ./root/etc/fstab $mountpath/etc/
# Put the latest binaries on the image
echo "Copying the latest AKLauncherDaemon from $BinariesDir"
cp $BinariesDir/AKLauncherDaemon $mountpath/usr/local/bin/
echo "Copying the latest AzureKinectNanoToFusion from $BinariesDir"
cp $BinariesDir/AzureKinectNanoToFusion $mountpath/usr/local/bin/
echo "Making the staging folder"
mkdir -p $mountpath/home/peabody/staging
echo "Moving in the readme"
cp ./root/home/peabody/staging/readme.md $mountpath/home/peabody/staging/
echo "Installing RunK4A.sh"
cp ./root/home/peabody/RunK4A.sh $mountpath/home/peabody/
echo "Installing the RenamePod helper script"
cp ./root/home/peabody/RenamePod.sh $mountpath/home/peabody/
echo "Cleaning up the log folder"
rm -rf $mountpath/var/log/K4AToFusion/*
echo "Installing the base 3DTelemedicine.cfg"
cp ../../ConfigFileExamples/3DTelemedicine.cfg $mountpath/home/peabody/K4AToFusion/

echo ""
echo "Done.  Hostname is now:"
cat $mountpath/etc/hostname
echo "Routing table set to:"
cat $mountpath/etc/hosts
echo "Network interface config set to:"
cat $mountpath/etc/network/interfaces
echo "Eth0 network interface config set to:"
cat $mountpath/etc/network/interfaces.d/eth0

# unmount
sudo umount $Device 
echo "Done.  If no errors were seen aside from 'umount: ...' everything worked!"


