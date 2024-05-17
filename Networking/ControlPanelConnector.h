#pragma once
#ifndef _MSC_VER
#include "Win2Linux.h"
#endif
#include "NetworkingDataTypes.h"
#include <thread>
#include <iostream>
#include <sstream>
#include <chrono>
#include "zmq.hpp"
#include "zmq_addon.hpp"

#include "Logger.h"

// You must use at least ONE of these transports
#undef USE_UDP
#define USE_TCP

#define SubscriberReconnectTimeoutSeconds 300
/* These should be passed into the ControlPanelConnector declaration,
and are read from the config.txt file
#define CONTROL_PANEL_IP ""
#define MY_INTERFACE_IP ""
#define CONTROL_PANEL_EVENT_TCP_PORT "tcp://" CONTROL_PANEL_IP ":62016"   // This object is subscriber, control panel is publisher. Put IP of control panel here
#define CONTROL_PANEL_EVENT_EPGM_PORT "epgm://"MY_INTERFACE_IP ";" CONTROL_PANEL_IP ":62017"   // This object is subscriber, control panel is publisher. Put IP of control panel here
// Note that with EPGM the standard is epgm://[INTERFACE_IP];[CONTROL_PANEL_IP]:[PORT] but on Windows if you omit INTERFACE_IP it chooses the default interface
#define CONTROL_PANEL_STATUS_TCP_PORT "tcp:// *:62018"  // This object is publisher, control panel is subscriber
#define CONTROL_PANEL_STATUS_EPGM_PORT "epgm://"CONTROL_PANEL_IP ";" MY_INTERFACE_IP ":62019"  // This object is publisher, control panel is subscriber
// Note that with EPGM the standard is epgm://[CONTROL_PANEL_IP];[INTERFACE_IP]:[PORT] but on Windows if you omit INTERFACE_IP it chooses the default interface
*/

namespace ControlPanelConnector
{
	class ControlPanelConnector
	{
	public:

		ControlPanelConnector(std::string controlPanelIP,
			std::string eventTCPPort,
			std::string myInterfaceIP,
			std::string statusTCPPort,
			bool disableHeartbeat = false);
		~ControlPanelConnector();

		virtual long EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize); // CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0);

		long SendStatusUpdate(CPC_STATUS status, const void* statusData = NULL, int dataSize = 0, Logger::Verbosity logger_level = Logger::Debug);

		// Utility functions
		void PrintEvent(CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0, Logger::Verbosity logger_level = Logger::Trace);
		void DeleteCalibrationData();

		void SetIdentificationString(std::string str);
		std::string GetIdentificationString();
		// Contains known status of Encoder, Distributor, Fusion, and Render
		// This gets updated in EventReceived, so make sure when you override EventReceived, you still first call ControlPanelConnector::EventReceived(....)
		CPC_STATUS SystemStatus[SOFTWARE::COUNT];
		char* JSONCalibrationData;
		std::chrono::system_clock::time_point lastReceived;

		void SetVersionInformation(const int major, const int minor, const int patch, const int commits, const std::string& branchName, const std::string& description, const std::string& sha1);
		void SendBuildVersion();

	protected:
		zmq::context_t* subscriber_context;
		zmq::context_t* publisher_context;

		zmq::socket_t* Subscriber;
		zmq::socket_t* Publisher;
		
		zmq::active_poller_t* EventPoller;

		std::thread HeartbeatThread;
		volatile bool ThreadRunning = false;

		void InitializeStatus();
		std::string controlPanelEventTCPPort;
		std::string controlPanelStatusTCPPort;

		std::string identificationString;
		bool DisableHeartbeat = false;

		int version_major = -1;
		int version_minor = -1;
		int version_patch = -1;
		int version_commits = 0;
		std::string version_description;
		std::string version_sha1;
		std::string version_branch_name;
	};

}
