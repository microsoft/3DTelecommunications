The staging folder is used to hold updated copies of AKLauncherDaemon, because we can't copy of AKLauncherDaemon directly, as it is always in-use when the system is running normally.
However, when the AKLauncherDaemon service is started or re-started, it first checks the staging folder, and copies over any AKLauncherDaemon binaries into /usr/local/bin
before launching the binary.  So updating AKLauncherDaemon requires copying the new binary into this folder, and then restarting the service with

sudo service aklauncherdaemon restart
