#!/bin/bash
echo "Starting.  Here is your local IP address"
ifconfig
# AKLauncherDaemon ControlPanelIP EventPort DaemonStatusPort [DaemonMode] [CaptureRegEx] [CalibrationRegEx]
AKLauncherDaemon /home/peabody/K4AToFusion/AKLauncherDaemon.config 2>/var/log/K4AToFusion/AKLauncherDaemon.error
