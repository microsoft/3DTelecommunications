#include "CalibrationSoftwareControlPanelConnector.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

CalibrationSoftwareControlPanelConnector::CalibrationSoftwareControlPanelConnector(std::string controlPanelIP,
	std::string eventPort,
	std::string myInterfaceIP,
	std::string statusPort) : ControlPanelConnector(controlPanelIP, eventPort, myInterfaceIP, statusPort)
{
	GotStartCommand = false;
	GotStopCommand = false;
}

CalibrationSoftwareControlPanelConnector::~CalibrationSoftwareControlPanelConnector()
{
}

HRESULT CalibrationSoftwareControlPanelConnector::EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize)
{
	ControlPanelConnector::EventReceived(eventType, eventData, dataSize);
	UINT machineID = 0;
	switch (eventType)
	{
	// No events need to be processed by this class for now
	default:
		break;
	}

	return S_OK;
}
