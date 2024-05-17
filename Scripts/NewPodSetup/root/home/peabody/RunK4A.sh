#!/bin/bash
export DISPLAY=:0
mv /home/peabody/staging/AKLauncherDaemon /usr/local/bin/AKLauncherDaemon
chmod +x /usr/local/bin/AKLauncherDaemon

AzureKinectFirmwareTool -r
AKLauncherDaemon

#rm /var/log/K4AToFusion/*
rm /var/lock/AKLauncherDaemon

