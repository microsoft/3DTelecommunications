#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <zmq.h>
#include <string.h>
#include <mutex>
// Proc stuff for tracking processes
#include "readproc.h"
#include <regex.h>
#include <signal.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/reboot.h>
#include <fcntl.h>

#include "Win2Linux.h"
#include "ControlPanelConnector.h"
#include "Config.h"

using namespace ControlPanelConnector;

class K4ARecorder :
	public ControlPanelConnector
{
public:
	K4ARecorder(std::string controlPanelIP,
		std::string eventTCPPort,
		std::string myInterfaceIP,
		std::string statusTCPPort,
		bool disableHeartbeat = false);
	~K4ARecorder();

	long EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0);
};

