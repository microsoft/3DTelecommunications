#pragma once
#include "ControlPanelConnector.h"
#include "../FusionDemo-MultiView/GlobalDataStatic.h"

using namespace ControlPanelConnector;

class FusionControlPanelConnector :
	public ControlPanelConnector
{
public:
	FusionControlPanelConnector(std::string controlPanelIP,
		std::string eventPort,
		std::string myInterfaceIP,
		std::string statusPort);
	~FusionControlPanelConnector();

	long EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0);
	bool GotStartCommand;
	bool GotStopCommand;

};
