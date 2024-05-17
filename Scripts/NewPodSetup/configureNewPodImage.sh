#!/bin/bash
# edit configureNewPodImage.sh to remove newlines
# use sed to get rid of invalid newline characters in configureNewPodImage.sh
sudo sed -i 's/\r//g' configureNewPodImage.sh

#test if /home/peabody/root exists
if [ ! -d /home/peabody/root ]; then
    echo "ERROR: /home/peabody/root does not exist.  Please copy the root folder to /home/peabody/ on the Nano before proceeding with this script"
    exit 1
fi
# verify that /home/peabody/staging exists
if [ ! -d /home/peabody/staging ]; then
    mkdir -p /home/peabody/staging
fi
# verify that /home/peabody/staging contains aklauncherdaemon 
if [ ! -f /home/peabody/staging/AKLauncherDaemon ]; then
    echo "ERROR: /home/peabody/staging/aklauncherdaemon does not exist.  Please copy the aklauncherdaemon binary to /home/peabody/staging/ on the Nano before proceeding with this script"
    exit 1
fi
# verify that /home/peabody/staging contains AzureKinectNanoToFusion
if [ ! -f /home/peabody/staging/AzureKinectNanoToFusion ]; then
    echo "ERROR: /home/peabody/staging/AzureKinectNanoToFusion does not exist.  Please copy the AzureKinectNanoToFusion binary to /home/peabody/staging/ on the Nano before proceeding with this script"
    exit 1
fi
# verify that /home/peabody/staging contains K4ARecorder
if [ ! -f /home/peabody/staging/K4ARecorder ]; then
    echo "ERROR: /home/peabody/staging/K4ARecorder does not exist.  Please copy the K4ARecorder binary to /home/peabody/staging/ on the Nano before proceeding with this script"
    exit 1
fi
# verify that /usr/lib/aarch64-linux-gnu/ contains libzmq.so.5.2.5
if [ ! -f /usr/lib/aarch64-linux-gnu/libzmq.so.5.2.5 ]; then
    # if not, check if it is in /home/peabody/root/usr/lib/aarch64-linux-gnu/
    # if it is, it will get copied automatically lower in the script
    if [ ! -f /home/peabody/root/usr/lib/aarch64-linux-gnu/libzmq.so.5.2.5 ]; then
        echo "ERROR: /usr/lib/aarch64-linux-gnu/libzmq.so.5.2.5 does not exist.  Please copy the libzmq.so.5.2.5 binary to /usr/lib/aarch64-linux-gnu/ on the Nano before proceeding with this script"
        exit 1
    fi
fi
# verify that opencv 4.5.5 is installed by looking for the existence of /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.5
if [ ! -f /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.5 ]; then
    # Verify the tarball with the correct version of opencv is in /home/peabody/staging
    if [ ! -f /home/peabody/staging/libopencv.4.5.5.tar.gz ]; then
        echo "ERROR: /home/peabody/staging/libopencv.4.5.5.tar.gz does not exist.  Please copy the libopencv.4.5.5.tar.gz binary to /home/peabody/staging/ on the Nano before proceeding with this script"
        exit 1
    fi
    # else, untar the opencv tarball at the root level
    cd /
    sudo tar -zxvf /home/peabody/staging/libopencv.4.5.5.tar.gz
fi

sudo apt install -y libglu1-mesa-dev freeglut3-dev mesa-common-dev openssl ninja-build libssl-dev libsoundio-dev libxinerama-dev libsdl2-dev curl
# uninstall Unity desktop, LXDE desktop, and OpenBox desktops
sudo apt-get remove --purge -y unity-session unity lxde openbox
sudo apt autoremove

curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/multiarch/prod 
sudo apt update
sudo apt install libk4a1.4 libk4a1.4-dev k4a-tools

cd ~

sudo mkdir /mnt/USB
sudo chmod a+rwx /mnt/USB

sudo cp -r /home/peabody/root/* /
sudo rm -rf /home/peabody/root  
sudo chmod a+x RunK4A.sh
sudo chmod a+x RenamePod.sh
sudo rm /etc/systemd/system/graphical.target.wants/aklauncherdaemon.service
sudo ln -s /etc/systemd/system/aklauncherdaemon.service /etc/systemd/system/graphical.target.wants/aklauncherdaemon.service
sudo systemctl enable aklauncherdaemon.service
sudo cp /etc/ntp.nanox.conf /etc/ntp.conf

#use sed to get rid of invalid newline characters in RunK4A.sh
sudo sed -i 's/\r//g' RunK4A.sh
sudo sed -i 's/\r//g' RenamePod.sh
sudo sed -i 's/\r//g' /etc/rc.local

sudo chown peabody:peabody /home/peabody/staging
sudo chown -R peabody:peabody /home/peabody/K4AToFusion
sudo mv /home/peabody/staging/AKLauncherDaemon /usr/local/bin/AKLauncherDaemon
sudo chmod a+rwx /usr/local/bin/AKLauncherDaemon
sudo mv /home/peabody/staging/AzureKinectNanoToFusion /usr/local/bin/AzureKinectNanoToFusion
sudo chmod a+rwx /usr/local/bin/AzureKinectNanoToFusion
sudo mv /home/peabody/staging/K4ARecorder /usr/local/bin/K4ARecorder
sudo chmod a+rwx /usr/local/bin/K4ARecorder
sudo chmod a+rwx /var/log/K4AToFusion


#reload the libraries in /usr/lib/aarch64-linux-gnu/
sudo ldconfig

# connect and accept the ssh-key of fusion (192.168.101.250) so we can scp to it
ssh-keyscan -H peabodycontrolpanel >> ~/.ssh/known_hosts

./RenamePod.sh 11

sudo chmod u+s /sbin/shutdown

#make user peabody auto-login to the desktop
# modify /etc/gdm3/custom.conf
sudo sed -i 's/#  AutomaticLoginEnable = true/AutomaticLoginEnable = true/g' /etc/gdm3/custom.conf
sudo sed -i 's/#  AutomaticLogin = user1/AutomaticLogin = peabody/g' /etc/gdm3/custom.conf

