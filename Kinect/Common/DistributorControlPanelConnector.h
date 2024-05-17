#pragma once
#include "ControlPanelConnector.h"

#define CAR_SERVER_IP "172.31.41.198"
#define MOCK_CAR_SERVER_IP "172.31.46.201"
#define ENCODER_SERVER_IP MOCK_CAR_SERVER_IP

using namespace ControlPanelConnector;

class DistributorControlPanelConnector :
	public ControlPanelConnector
{
public:
	DistributorControlPanelConnector();
	~DistributorControlPanelConnector();

	HRESULT EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0);
	bool GotStartCommand;
	bool GotStopCommand;

	char* EncoderIP;
};

