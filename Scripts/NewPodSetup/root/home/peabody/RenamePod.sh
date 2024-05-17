#!/bin/bash
# Hostname to add/remove.
POD_NUMBER=$1
CURRENTHOSTNAME=`cat /etc/hostname`
HOSTNAME=POD-$POD_NUMBER
IP=192.168.101.$POD_NUMBER

echo "Changing hostname to $HOSTNAME"
echo "Changing IP to $IP"

# Replace the first instance in hosts
sudo sed -i".bak" "2 s/$CURRENTHOSTNAME/$HOSTNAME/" /etc/hosts
sudo sed -i".bak" "1 s/$CURRENTHOSTNAME/$HOSTNAME/" /etc/hostname
sudo sed -i".bak" "3 s/192.168.101.[0-9]*/$IP/" /etc/network/interfaces.d/eth0

cat /etc/hosts
cat /etc/hostname
cat /etc/network/interfaces.d/eth0

