#include "ControlPanelConnector.h"
#include <mutex>
#include "Logger.h"

std::mutex zmqPubMutx;
std::mutex zmqSubMutx;
#define IS_ALIVE_INTERVAL_MSEC 10000

namespace ControlPanelConnector
{
	ControlPanelConnector::ControlPanelConnector(std::string controlPanelIP,
		std::string eventTCPPort,
		std::string myInterfaceIP,
		std::string statusTCPPort,
		bool disableHeartbeat) : identificationString(), version_description("Uninitialized"), version_sha1("Uninitialized"), version_branch_name("uninitialized")
	{
		DisableHeartbeat = disableHeartbeat;
		controlPanelEventTCPPort = "tcp://" + controlPanelIP + ":" + eventTCPPort;   // This object is subscriber, control panel is publisher. Put IP of control panel here
		controlPanelStatusTCPPort = "tcp://*:" + statusTCPPort;  // This object is publisher, control panel is subscriber
		InitializeStatus();
		// Initialize a zmq Publisher to send status updates
		publisher_context = new zmq::context_t(1);
		Publisher = new zmq::socket_t(*publisher_context, ZMQ_PUB);
		// Initialize a zmq Subscriber to receive events
		subscriber_context = new zmq::context_t(1);
		
		Subscriber = new zmq::socket_t(*subscriber_context, ZMQ_SUB);
		EventPoller = new zmq::active_poller_t();

		Subscriber->set(zmq::sockopt::subscribe, ""); // Subscribe to ALL events
		Subscriber->set(zmq::sockopt::rcvtimeo, ZMQ_REQUEST_TIMEOUT);
		Subscriber->set(zmq::sockopt::tcp_keepalive, 1);
		Subscriber->set(zmq::sockopt::tcp_keepalive_cnt, 2);
		Subscriber->set(zmq::sockopt::tcp_keepalive_idle, 10);
		Subscriber->set(zmq::sockopt::tcp_keepalive_intvl, 10);
		Subscriber->set(zmq::sockopt::rcvbuf, 1000000);
		Subscriber->set(zmq::sockopt::sndbuf, 1000000);
		Subscriber->set(zmq::sockopt::sndhwm, 10000);
		Subscriber->set(zmq::sockopt::rcvhwm, 10000);
#ifdef USE_TCP
		try{
			*LOGGER() << Logger::Info << "Publishing on " << controlPanelStatusTCPPort << Logger::endl;
			Publisher->bind(controlPanelStatusTCPPort);
			*LOGGER() << Logger::Info << "Subscribing to " << controlPanelEventTCPPort << Logger::endl;
			Subscriber->connect(controlPanelEventTCPPort);
		}
		catch (const std::exception& e)
		{
			LOGGER()->error("ControlPanelConnector::ControlPanelConnector", "Error setting up ZMQ sockets: %s", e.what());
			return;
		}
#endif
		EventPoller->add(*Subscriber, zmq::event_flags::pollin, [this](zmq::event_flags ef) 
			{
				zmq::message_t message;

				zmqSubMutx.lock();
				auto haveData = this->Subscriber->recv(message, zmq::recv_flags::none);
				zmqSubMutx.unlock();
				if (!haveData.has_value())
				{
					// No data
					return;
				}

				std::istringstream iss(static_cast<char*>(message.data()));
				CONTROL_PANEL_EVENT eventType = (CONTROL_PANEL_EVENT)((char*)message.data())[0];
				void* eventData = (void*)message.data();
				int dataSize = message.size();
				this->EventReceived(eventType, eventData, dataSize);
			});
		JSONCalibrationData = nullptr;

		HeartbeatThread = std::thread([this]()
		{
			int blips = 0;
			ThreadRunning = true;
			if(DisableHeartbeat)
			{
				LOGGER()->info("Disabling heartbeat thread per config DisableHearbeat value.");
				return;
			}
			while (ThreadRunning)
			{
				if(blips == 0)
				{
					if(!identificationString.empty())
					{
						SendStatusUpdate(CPC_STATUS::IS_ALIVE, identificationString.c_str(), identificationString.length(), Logger::Verbosity::Trace);
					}
					else
					{
						SendStatusUpdate(CPC_STATUS::IS_ALIVE, NULL, 0, Logger::Verbosity::Trace);				
					}
					blips++;
				}
				auto n = EventPoller->wait(std::chrono::milliseconds(IS_ALIVE_INTERVAL_MSEC));
				// Only increment the blips if we timed out (not new data) 
				if(!n)
					blips = blips == 100?0:blips+1;

				// Also reconnect the subscriber if it hasn't heard anything from the publisher in a while
				int elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - lastReceived).count();
				if(elapsedSeconds > SubscriberReconnectTimeoutSeconds)
				{
					*LOGGER() << Logger::Debug << "Subscriber connection to " << controlPanelEventTCPPort << " appears stale.  Reconnecting." << Logger::endl;
					
					std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
					zmqSubMutx.lock();
					int dur = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
					*LOGGER() << Logger::Debug << "Acquiring lock took " << dur << "sec" << Logger::endl;

					start = std::chrono::system_clock::now();					
					Subscriber->disconnect(controlPanelEventTCPPort);
					dur = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
					*LOGGER() << Logger::Debug << "Disconnect took " << dur << "sec" << Logger::endl;
					
					start = std::chrono::system_clock::now();
					Subscriber->set(zmq::sockopt::subscribe, "");// Subscribe to ALL events
					Subscriber->set(zmq::sockopt::rcvtimeo, ZMQ_REQUEST_TIMEOUT);
					Subscriber->connect(controlPanelEventTCPPort);
					dur = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
					*LOGGER() << Logger::Debug << "Connect took " << dur << "sec" << Logger::endl;
					
					zmqSubMutx.unlock();
					*LOGGER() << Logger::Debug << "Reconnection done. " << Logger::endl;
					lastReceived = std::chrono::system_clock::now();
					*LOGGER() << Logger::Debug << "Done reconnecting.  Took " << (float)(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count()) << "sec" << Logger::endl;
				}
			}
			LOGGER()->info("Heartbeat thread terminating.");
		});
	}

	ControlPanelConnector::~ControlPanelConnector()
	{
		DeleteCalibrationData();
		ThreadRunning = false;
		HeartbeatThread.join();

		Publisher->close();
		Subscriber->close();

		delete publisher_context;
		delete Publisher;
		delete subscriber_context;
		delete Subscriber;
		delete EventPoller;
	}

	void ControlPanelConnector::InitializeStatus()
	{
		for (int i = 0; i < SOFTWARE::COUNT; i++)
		{
			SystemStatus[i] = CPC_STATUS::STOPPED;
		}
	}

	void ControlPanelConnector::DeleteCalibrationData()
	{
		if(JSONCalibrationData != nullptr)
			delete JSONCalibrationData;
		JSONCalibrationData = nullptr;
	}
	void ControlPanelConnector::SetVersionInformation(const int major, const int minor, const int patch, const int commits, const std::string& branchName, const std::string& description, const std::string& sha1)
	{
		version_major = major;
		version_minor = minor;
		version_patch = patch;
		version_commits = commits;
		version_branch_name.assign(branchName);
		version_description.assign(description);
		version_sha1.assign(sha1);

		// Send it out when set
		SendBuildVersion();
	}
	void ControlPanelConnector::PrintEvent(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize, Logger::Verbosity verbosity)
	{
		LOGGER()->log(verbosity,"[ControlPanelConnector] Received an event!  Event Type:%d", eventType);
		switch (eventType)
		{
			// Ready/Start/Stop
		case CONTROL_PANEL_EVENT::DEPTH_GEN_STARTED:
			LOGGER()->log(verbosity,"DEPTH GEN IS STARTED");
			break;
		case CONTROL_PANEL_EVENT::DEPTH_GEN_STOPPED:
			LOGGER()->log(verbosity,"DEPTH GEN IS STOPPED");
			break;
		case CONTROL_PANEL_EVENT::FUSION_STARTED:
			LOGGER()->log(verbosity,"FUSION IS STARTED");
			break;
		case CONTROL_PANEL_EVENT::FUSION_STOPPED:
			LOGGER()->log(verbosity,"FUSION IS STOPPED");
			break;
		case CONTROL_PANEL_EVENT::RENDER_STARTED:
			LOGGER()->log(verbosity,"RENDER IS STARTED");
			break;
		case CONTROL_PANEL_EVENT::RENDER_STOPPED:
			LOGGER()->log(verbosity,"RENDER IS STOPPED");
			break;
			// While-running updates
		case CONTROL_PANEL_EVENT::DEPTH_GEN_FPS:
			LOGGER()->log(verbosity,"DEPTH FPS IS %f", *(float*)eventData);
			break;
		case CONTROL_PANEL_EVENT::FUSION_FPS:
			LOGGER()->log(verbosity,"FUSION FPS IS %f", *(float*)eventData);
			break;
		case CONTROL_PANEL_EVENT::RENDER_FPS:
			LOGGER()->log(verbosity,"RENDER FPS IS %f", *(float*)eventData);
			break;
			// While-running requests
		case CONTROL_PANEL_EVENT::SPEED_REQUEST:
			LOGGER()->log(verbosity,"SPEED REQUEST FOR SOFTWARE ID %d", *(int*)eventData);
			break;
		case CONTROL_PANEL_EVENT::CONTROL_PANEL_START_REQUESTED:
			LOGGER()->log(verbosity,"CONTROL PANELS IS REQUESTING A START");
			break;
		case CONTROL_PANEL_EVENT::CONTROL_PANEL_STOP_REQUESTED:
			LOGGER()->log(verbosity,"CONTROL PANEL IS REQUESTIN A STOP");
			break;
		case CONTROL_PANEL_EVENT::CONTROL_PANEL_STATE_UPDATE_REQUESTED:
			LOGGER()->log(verbosity,"CONTROL PANEL IS REQUESTING A STATE UPDATE");
			break;
		// Config data
		case CONTROL_PANEL_EVENT::CALIBRATION_DATA:
			LOGGER()->log(verbosity,"CONTROL PANEL IS SENDING CALIBRATION DATA");
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_CAPTURE_FRAME:
			LOGGER()->log(verbosity,"CONTROL PANEL IS ASKING CALIBRATION TO CAPTURE A FRAME");
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_DOWNLOAD_OLDEST:
			LOGGER()->log(verbosity,"CONTROL PANEL IS REQUESTING TO DOWNLOAD THE OLDEST CAPTURED FRAME");
			break;		
		case CONTROL_PANEL_EVENT::KINECT_FACTORY_CALIBRATION_DATA_REQUESTED:
			LOGGER()->log(verbosity, "CONTROL PANEL IS PUBLISHING KNOWN EVENT: KINECT_FACTORY_CALIBRATION_DATA_REQUESTED");
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_STARTED:
			LOGGER()->log(verbosity, "CONTROL PANEL IS RE-BROADCASTING CALIBRATION SOFTWARE STARTED");
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED:
			LOGGER()->log(verbosity, "CONTROL PANEL IS RE-BROADCASTING CALIBRATION SOFTWARE CAPTURING VIDEO STARTED");
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO:
			LOGGER()->log(verbosity, "CONTROL PANEL IS RE-BROADCASTING CALIBRATION SOFTWARE TRANSFERRING VIDEO");
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE:
			LOGGER()->log(verbosity, "CONTROL PANEL IS RE-BROADCASTING CALIBRATION SOFTWARE VIDEO TRANSFER DONE");
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_PROCESSING:
			LOGGER()->log(verbosity, "CONTROL PANEL IS RE-BROADCASTING CALIBRATION SOFTWARE PROCESSING");
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_RESULT:
			LOGGER()->log(verbosity, "CONTROL PANEL IS RE-BROADCASTING CALIBRATION SOFTWARE RESULT");
			break;
		case CONTROL_PANEL_EVENT::SYSTEM_CONFIGURATION_UPDATE:
			LOGGER()->log(verbosity, "CONTROL PANEL IS PUBLISHING KNOWN EVENT: SYSTEM_CONFIGURATION_UPDATE");
			break;
		case CONTROL_PANEL_EVENT::LOG_COLLECTION_REQUESTED:
			LOGGER()->log(verbosity, "CONTROL PANEL IS PUBLISHING KNOWN EVENT: LOG_COLLECTION_REQUESTED");
			break;
		case CONTROL_PANEL_EVENT::NEW_BINARY_TRANSFER:
			LOGGER()->log(verbosity, "CONTROL PANEL IS PUBLISHING KNOWN EVENT: NEW_BINARY_TRANSFER");
			break;
		case CONTROL_PANEL_EVENT::TOGGLE_FUSION_HIGH_RESOLUTION:
			LOGGER()->log(verbosity, "CONTROL PANEL IS PUBLISHING KNOWN EVENT: TOGGLE_FUSION_HIGH_RESOLUTION");
			break;
		case CONTROL_PANEL_EVENT::BUILD_VERSION_REQUESTED:
			LOGGER()->log(verbosity, "CONTROL PANEL IS PUBLISHING KNOWN EVENT: BUILD_VERSION_REQUESTED");
			break;

		default:
			// To PR reviewer: should we always set the default case for unknown events to Warning? I think so...
			LOGGER()->log(Logger::Warning,"GOT SOME OTHER EVENT: %d", eventType);
			break;
		}
	}

	// Handles received events in a default way, this function can (and should) be overridden by a subclass that properly handles events
	// NOTE that the first byte of eventData will be the eventType
	long ControlPanelConnector::EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize)
	{

		lastReceived = std::chrono::system_clock::now();
		PrintEvent(eventType, eventData, dataSize);
		switch (eventType)
		{
		case CONTROL_PANEL_EVENT::DEPTH_GEN_STARTED:
			SystemStatus[SOFTWARE::CAPTURE] = CPC_STATUS::RUNNING;
			break;
		case CONTROL_PANEL_EVENT::DEPTH_GEN_STOPPED:
			SystemStatus[SOFTWARE::CAPTURE] = CPC_STATUS::STOPPED;
			break;
		case CONTROL_PANEL_EVENT::FUSION_STARTED:
			SystemStatus[SOFTWARE::FUSION] = CPC_STATUS::RUNNING;
			break;
		case CONTROL_PANEL_EVENT::FUSION_STOPPED:
			SystemStatus[SOFTWARE::FUSION] = CPC_STATUS::STOPPED;
			break;
		case CONTROL_PANEL_EVENT::RENDER_STARTED:
			SystemStatus[SOFTWARE::RENDER] = CPC_STATUS::RUNNING;
			break;
		case CONTROL_PANEL_EVENT::RENDER_STOPPED:
			SystemStatus[SOFTWARE::RENDER] = CPC_STATUS::STOPPED;
			break;
		case CONTROL_PANEL_EVENT::CONTROL_PANEL_START_REQUESTED:
		case CONTROL_PANEL_EVENT::CONTROL_PANEL_STOP_REQUESTED:
		case CONTROL_PANEL_EVENT::CONTROL_PANEL_STATE_UPDATE_REQUESTED:
			break;
		case CONTROL_PANEL_EVENT::CALIBRATION_DATA:
			LOGGER()->log(Logger::Info,"Got new calibration data.  Storing.");
			if (JSONCalibrationData != nullptr)
			{
				// we got a new one before we used the last one, so delete it first
				DeleteCalibrationData();
			}
			{
				int jsonPacketSize = ((int*)eventData)[0];
				JSONCalibrationData = new char[jsonPacketSize * sizeof(char)];
				memcpy_s(JSONCalibrationData, jsonPacketSize, (char*)eventData + sizeof(int), dataSize - sizeof(int));
			}
			// Note that this is deleted by Fusion after the data is processed.
			break;
		case CONTROL_PANEL_EVENT::BUILD_VERSION_REQUESTED:
		{
			SendBuildVersion();
		}
			break;
		default:
			PrintEvent(eventType, eventData, dataSize, Logger::Warning);
			break;
		}
		return 0;
	}	

	void ControlPanelConnector::SendBuildVersion()
	{
		// send a standard version packet back to the control panel
		//[CPC_STATUS::VERSION][(int)Major][(int)Minor][(int)Patch][(int)Commits][(int)branchNameLength][(string[branchNameLength)branchName][(int)descriptionLength][(string[descriptionLength])description][(string[BUILD_VERSION_SHA1_LENGTH])sha1]
		int packetSize = 6 * sizeof(int) + version_branch_name.length() + version_description.length() + BUILD_VERSION_SHA1_LENGTH;
		char* data = (char*)malloc(packetSize * sizeof(char));
		char* ptr = data;
		*((int*)ptr) = version_major;
		ptr += sizeof(int);
		*((int*)ptr) = version_minor;
		ptr += sizeof(int);
		*((int*)ptr) = version_patch;
		ptr += sizeof(int);
		*((int*)ptr) = version_commits;
		ptr += sizeof(int);
		*((int*)ptr) = version_branch_name.length();
		ptr += sizeof(int);
		memcpy(ptr, version_branch_name.c_str(), version_branch_name.length());
		ptr += version_branch_name.length();
		*((int*)ptr) = version_description.length();
		ptr += sizeof(int);
		memcpy(ptr, version_description.c_str(), version_description.length());
		ptr += version_description.length();
		memcpy(ptr, version_sha1.c_str(), BUILD_VERSION_SHA1_LENGTH);

		LOGGER()->info("Build version %d.%d.%d+%d %s (%s)", version_major, version_minor, version_patch, version_commits, version_description.c_str(), version_sha1.c_str());

		SendStatusUpdate(CPC_STATUS::BUILD_VERSION, data, packetSize);
		free(data);
	}

	// Sends a status update to the control panel
	long ControlPanelConnector::SendStatusUpdate(CPC_STATUS status, const void* statusData, int dataSize, Logger::Verbosity verbosity)
	{		
		*LOGGER() << verbosity << "[Control Panel Connector] Sending status update: " << (int)status << " message size: " << dataSize << " bytes " << Logger::endl;

		char* data = (char*)malloc((dataSize + 1) * sizeof(char));
		data[0] = (char)status;
		if (dataSize > 0 && statusData != NULL)
		{
			memcpy_s(&data[1], dataSize, statusData, dataSize);
			// This is ABOVE trace, because it puts out a LOT of output
			if(verbosity > Logger::Trace)
			{
				for(int i = 0; i < dataSize; i++)
				{
					LOGGER()->trace("Byte[%d] %0x ", i, ((char*)statusData)[i]);
				}
			}
		}
		zmq::message_t message = zmq::message_t(data, dataSize + 1);

		zmq::send_result_t sendResults;
		zmqPubMutx.lock();
		{
			sendResults = Publisher->send(message, zmq::send_flags::none);
		}
		zmqPubMutx.unlock();
		
		free(data);
		if (!sendResults.has_value())
		{
			return -1;
		}		
		return 0;
	}

	// if identification string is set, we will transmit it with the heartbeat signal
	void ControlPanelConnector::SetIdentificationString(std::string str)
	{
		identificationString = str;
	}

	std::string ControlPanelConnector::GetIdentificationString()
	{
		return identificationString;
	}

}
