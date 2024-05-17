#include "K4AControlPanelConnector.h"


K4AControlPanelConnector::K4AControlPanelConnector(std::string controlPanelIP,
	std::string eventTCPPort,
	std::string myInterfaceIP,
	std::string statusTCPPort,
	bool disableHeartbeat) : ControlPanelConnector(controlPanelIP,
		eventTCPPort,
		myInterfaceIP,
		statusTCPPort,
		disableHeartbeat)
{	
	SetStopCommand(false);
	SetStartCommand(false);
	SetCalibrationFrameRequest(false);
	SetCalibrationCaptureFrame(false);
	SetHasNewCalibrationData(false);
	SetRequestedSpeed(-1);
}

bool K4AControlPanelConnector::GetStopCommand()
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	return StopCommand;
}
bool K4AControlPanelConnector::GetStartCommand()
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	return StartCommand;
}
bool K4AControlPanelConnector::GetCalibrationFrameRequest()
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	return CalibrationFrameRequest;
}
bool K4AControlPanelConnector::GetCalibrationCaptureFrame()
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	return CalibrationCaptureFrame;
}
bool K4AControlPanelConnector::GetHasNewCalibrationData()
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	return HasNewCalibrationData;
}
int K4AControlPanelConnector::GetRequestedSpeed()
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	return RequestedSpeed;
}


Calibration K4AControlPanelConnector::GetCalibration(std::string serial)
{
	if(calibrations.find(serial) == calibrations.end())
	{
		std::cout << "K4ACPC:GetCalibration -- NO CALIBRATION DATA FOUND FOR " << serial << std::endl;
	}
	return calibrations[serial];
}

void K4AControlPanelConnector::SetStopCommand(bool val)
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	StopCommand = val;
}
void K4AControlPanelConnector::SetStartCommand(bool val)
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	StartCommand = val;
}
void K4AControlPanelConnector::SetCalibrationFrameRequest(bool val)
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	CalibrationFrameRequest = val;
}
void K4AControlPanelConnector::SetCalibrationCaptureFrame(bool val)
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	CalibrationCaptureFrame = val;
}
void K4AControlPanelConnector::SetKinectSerialNumber(std::string& serial)
{
	SetIdentificationString(serial);
}
void K4AControlPanelConnector::SetHasNewCalibrationData(bool val)
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	HasNewCalibrationData = val;
}
void K4AControlPanelConnector::SetRequestedSpeed(int speed)
{
	std::lock_guard<std::mutex> lck(eventWriteMutex);
	RequestedSpeed = speed;
}

K4AControlPanelConnector::~K4AControlPanelConnector()
{
}

// global start/stop
// distributor slowdown
// possibly distributor FPS
long K4AControlPanelConnector::EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize)
{
	ControlPanelConnector::EventReceived(eventType, eventData, dataSize);
	switch (eventType)
	{
	case CONTROL_PANEL_EVENT::DEPTH_GEN_STARTED:
	case CONTROL_PANEL_EVENT::DEPTH_GEN_STOPPED:
	case CONTROL_PANEL_EVENT::FUSION_STARTED:
	case CONTROL_PANEL_EVENT::FUSION_STOPPED:

	case CONTROL_PANEL_EVENT::DEPTH_GEN_FPS:
	case CONTROL_PANEL_EVENT::FUSION_FPS:
	case CONTROL_PANEL_EVENT::RENDER_FPS:
		break;
	case CONTROL_PANEL_EVENT::SPEED_REQUEST:
		printf("Got a speed request.\n");
		if(dataSize < (int)(sizeof(char) + sizeof(int) + sizeof(int)))
		{
			printf("Error. Got a speed request without a requested speed or software!  Ignoring.\n");
		}
		else
		{
			char* data = (char*)eventData;
			data++; // skip the event type
			int software = *(int*)data;
			data += sizeof(int);
			int requestedSpeed = *(int*)data;			
			printf("Got a speed request for software %d of %d fps\n", software, requestedSpeed);
			if((SOFTWARE)software == SOFTWARE::CAPTURE)
			{
				//Quick validity check
				if(requestedSpeed > MaxKinectFramesPerSecond)
				{
					printf("That speed request is invalid.  Ignoring.\n");
				}
				else
				{
					SetRequestedSpeed(requestedSpeed);
				}
			}
		}
		break;
	case CONTROL_PANEL_EVENT::CONTROL_PANEL_START_REQUESTED:
        if(dataSize == 1)
        {
    		printf("~~~~~~~Got a system start request!~~~~~~~~\n");
            SetStartCommand(true);
        }
        else
        {
            printf("Ignoring system start request because it has a data packet (%d), so it is a start application request.  Not a start capture.\n", dataSize);
        }
		break;
	case CONTROL_PANEL_EVENT::CONTROL_PANEL_STOP_REQUESTED:
        if(dataSize == 1)
        {
            printf("~~~~~~~Got a system stop request!~~~~~~~~~~\n");
            SetStopCommand(true);
        }
        else
        {
            printf("Ignoring system stop request because it has a data packet (%d), so it is a start application request.  Not a stop capture.\n", dataSize);
        }
		break;
	case CONTROL_PANEL_EVENT::CALIBRATION_DATA:
		// Data gets saved by parent class in JSONCalibrationData
		if(dataSize > 0)
		{
			int jsonPacketSize = ((int*)eventData)[0];
			std::cout << "Got new calibration data.  Packet size is " << jsonPacketSize << std::endl;
			Json::Value root;
			Json::Value nullValue;
			Json::Reader reader;
			char* jsonStart = JSONCalibrationData + sizeof(char);
			std::string dataString(jsonStart, jsonPacketSize);
			
			std::cout << "parsing JSON" << std::endl;
			if(!reader.parse(dataString, root, false))
			{
				std::cout << "Error parsing the JSON!" << std::endl;
				std::cout << std::endl << std::endl << reader.getFormattedErrorMessages() << std::endl;
				std::cout << std::endl << std::endl << dataString << std::endl << std::endl;				
			}
			std::cout << "done." << std::endl;

			for(Json::Value::const_iterator iter = root.begin(); iter != root.end(); iter++)
			{
				Json::Value node = *iter;
				Calibration calibration;
				calibration.zScale = 1.0;
				calibration.zShift = 0.0;
				calibration.width = node.get("Width", 1920).asInt();
				calibration.height = node.get("Height", 1080).asInt();
				
				calibration.K[0] = node.get("Fx", 0.0).asDouble();
				calibration.K[1] = node.get("Fy", 0.0).asDouble();
				calibration.K[2] = node.get("Px", 0.0).asDouble();
				calibration.K[3] = node.get("Py", 0.0).asDouble();
				
				calibration.numKParams = node.get("IsR6kt", false).asBool()?6:3;

				if(!node.isMember("Kappa"))
				{
					std::cout << "ERROR parsing JSON.  NO Kappa array found!" << std::endl;
					break;
				}				
				Json::Value kappaNode = node.get("Kappa", nullValue);
				if(!kappaNode.isArray())
				{
					std::cout << "ERROR. Kappa isn't an array!" << std::endl;
					break;
				}
				for(int k = 0; k <= calibration.numKParams; k++)
				{
					//std::cout << "Kappa[" << k << "] (" << kappaNode[k].type() << ") = " << kappaNode[k].toStyledString() << std::endl;
					calibration.dist[k] = (kappaNode[k]).asDouble();
				}
				
				double rodrigues[3];
				if(!node.isMember("Rotation"))
				{
					std::cout << "ERROR parsing JSON.  No Rotation array found!" << std::endl;
					break;
				}
				Json::Value rotation = node.get("Rotation", nullValue);
				rodrigues[0] = rotation.get("x", nullValue).asDouble();
				rodrigues[1] = rotation.get("y", nullValue).asDouble();
				rodrigues[2] = rotation.get("z", nullValue).asDouble();
				RodriguesToRotation(rodrigues[0], rodrigues[1], rodrigues[2], calibration.R);
				
				if(!node.isMember("Translation"))
				{
					std::cout << "ERROR parsing JSON.  No Translation array found!" << std::endl;
					break;
				}
				Json::Value translation = node.get("Translation", nullValue);
				calibration.t[0] = translation.get("x", nullValue).asDouble();
				calibration.t[1] = translation.get("y", nullValue).asDouble();
				calibration.t[2] = translation.get("z", nullValue).asDouble();

				if(!node.isMember("ColorScale"))
				{
					std::cout << "ERROR parsing JSON.  No ColorScale array found!" << std::endl;
					break;
				}
				Json::Value colorScale = node.get("ColorScale", nullValue);
				calibration.colorScale[0] = colorScale.get("x", nullValue).asDouble();
				calibration.colorScale[1] = colorScale.get("y", nullValue).asDouble();
				calibration.colorScale[2] = colorScale.get("z", nullValue).asDouble();

				if(!node.isMember("ColorBias"))
				{
					std::cout << "ERROR parsing JSON.  No ColorBias array found!" << std::endl;
					break;
				}
				Json::Value colorBias = node.get("ColorBias", nullValue);
				calibration.colorBias[0] = colorBias.get("x", nullValue).asDouble();
				calibration.colorBias[1] = colorBias.get("y", nullValue).asDouble();
				calibration.colorBias[2] = colorBias.get("z", nullValue).asDouble();

				std::string name = node.get("Name", "NameNotFound").asString();
				std::cout << "Successfully loaded new calibration for " << name << ".  Saving to disk." << std::endl;
				SaveCamParametersCalibStudio("./K4AToFusion", name.c_str(), calibration);
				calibrations.insert_or_assign(std::ref(name), std::ref(calibration));
				SendStatusUpdate(CPC_STATUS::RECEIVED_NEW_DATA, NULL, 0);
			}
			SetHasNewCalibrationData(true);			
			DeleteCalibrationData();	
		}
		break;
	case CONTROL_PANEL_EVENT::CALIBRATION_CAPTURE_FRAME:
		std::cout << "Got a capture frame request" << std::endl;
		SetCalibrationCaptureFrame(true);
		break;
	case CONTROL_PANEL_EVENT::CALIBRATION_DOWNLOAD_OLDEST:
		std::cout << "Got a download frame request" << std::endl;
		SetCalibrationFrameRequest(true);
		break;
	default:
		break;
	}

	return S_OK;
}
