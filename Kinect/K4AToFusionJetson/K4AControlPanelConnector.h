#pragma once
#include "ControlPanelConnector.h"
#include <mutex>
#include <json/json.h>
#include "K4ACommon.h"
#include "Util.h"

using namespace ControlPanelConnector;

class K4AControlPanelConnector :
	public ControlPanelConnector
{
public:
	K4AControlPanelConnector(std::string controlPanelIP,
		std::string eventTCPPort,
		std::string myInterfaceIP,
		std::string statusTCPPort,
		bool disableHeartbeat = false);
	~K4AControlPanelConnector();

	long EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0);
	bool GetStartCommand();
	bool GetStopCommand();
	bool GetCalibrationFrameRequest();
	bool GetCalibrationCaptureFrame();
	bool GetHasNewCalibrationData();
	Calibration GetCalibration(std::string serial);
	int GetRequestedSpeed();

	void SetStartCommand(bool val);
	void SetStopCommand(bool val);
	void SetCalibrationFrameRequest(bool val);
	void SetCalibrationCaptureFrame(bool val);
	void SetHasNewCalibrationData(bool val);
	void SetKinectSerialNumber(std::string& serial);
	void SetCalibrationFilePath(std::string& path);
	void SetRequestedSpeed(int speed);
protected:
	std::mutex eventWriteMutex;
	bool StartCommand;
	bool StopCommand;
	bool CalibrationFrameRequest;
	bool CalibrationCaptureFrame;
	bool HasNewCalibrationData;
	int RequestedSpeed;
	std::map<std::string, Calibration> calibrations;
	std::string CalibrationFilePath;
};

