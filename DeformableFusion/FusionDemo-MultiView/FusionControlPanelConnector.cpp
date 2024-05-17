#include "FusionControlPanelConnector.h"
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

FusionControlPanelConnector::FusionControlPanelConnector(std::string controlPanelIP,
	std::string eventPort,
	std::string myInterfaceIP,
	std::string statusPort) : ControlPanelConnector(controlPanelIP, eventPort, myInterfaceIP, statusPort)
{
	GotStartCommand = false;
	GotStopCommand = false;
}

FusionControlPanelConnector::~FusionControlPanelConnector()
{
}

HRESULT FusionControlPanelConnector::EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize)
{
	ControlPanelConnector::EventReceived(eventType, eventData, dataSize);
	UINT machineID = 0;
	switch (eventType)
	{
	case CONTROL_PANEL_EVENT::RENDER_FPS:
		LOGGER()->info("Got render FPS: %f", ((float*)((char*)eventData+1))[0]);
		break;

	case CONTROL_PANEL_EVENT::SPEED_REQUEST:
		LOGGER()->info("Component is issuing a speed request");
		break;

	case CONTROL_PANEL_EVENT::CONTROL_PANEL_START_REQUESTED:
		// I don't actually use this value.  The distributor just waits until it has an encoder to connect to and then starts listening.  Depth gen clients can technically connect at any time
		GotStartCommand = true;
		LOGGER()->info("Got a system start request!");
		break;
	case CONTROL_PANEL_EVENT::CONTROL_PANEL_STOP_REQUESTED:
		// Dont respond to this in new version of fusion, since the fusion service running on the same machine will monitor that and issue kills here if necessary 
		LOGGER()->info("Got a system stop request!");
		break;
	case CONTROL_PANEL_EVENT::TOGGLE_FUSION_HIGH_RESOLUTION:
		{
			LOGGER()->info("Got a TOGGLE FUSION QUALITY PACKET!");
			std::lock_guard<std::mutex> lock_cuda_res(GlobalDataStatic::mutex_cuda);
			if (GlobalDataStatic::bStopFusion)
			{
				GlobalDataStatic::bStopFusion = false;
				GlobalDataStatic::bNextFrameHighQuality = false;
			}
			else
			{
				GlobalDataStatic::bNextFrameHighQuality = true;
			}
			break;
		}
	default:
		break;
	}

	return S_OK;
}
