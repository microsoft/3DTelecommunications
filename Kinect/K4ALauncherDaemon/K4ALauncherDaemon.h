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

#include <k4a/k4a.h>
#include "Logger.h"

using namespace ControlPanelConnector;

#define SOFTWARE_STATES	3  //how many states are we tracking?

class K4ALauncherDaemon :
	public ControlPanelConnector
{
public:
	K4ALauncherDaemon(std::string controlPanelIP,
		std::string eventTCPPort,
		std::string myInterfaceIP,
		std::string statusTCPPort,
		bool disableHeartbeat = false);
	~K4ALauncherDaemon();

	long EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0);
	static void StateMonitor(K4ALauncherDaemon* daemon);
	static void ApplicationThread(K4ALauncherDaemon* daemon);
	void PrintStates();
	void SetState(int idx, SOFTWARE_STATE state);
	SOFTWARE_STATE GetState(int idx);
	void GetAllStates(SOFTWARE_STATE (&states)[SOFTWARE::COUNT]);
	void SetPID(int idx, int pid);
	int GetPID(int idx);
	void SendSoftwareStateUpdate();
	void KillSoftware(int idx, int signal);
	bool AllApplicationsStopped();
	void SetConfig(CConfig* config);
	void SendLogFile(std::string filename);
	void SendInstallationResult(SOFTWARE software, bool success, std::string errorMessage);
	void Cleanup(int signal);
	int Verbosity;
	std::string serialPortFilename;

protected:
	SOFTWARE_STATE SoftwareStates[SOFTWARE::COUNT];
	int PIDs[SOFTWARE::COUNT];
	std::thread stateMonitoringThread;
	std::mutex stateMutex;

	std::thread applicationThread;
	
	bool runThread;
	void PrintState(SOFTWARE_STATE state);

	CConfig* config;

	const int numRetriesBeforeSigTerm = 2;

	std::string kinectSerialNumber;
	void GetKinectSerialNumber();
	bool calibrationSoftwarePart2Running;
	bool SetStateIfSerialNumberMatches(void* eventData, int dataSize, std::string stateString);
};

