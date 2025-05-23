// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#pragma once
#include "ControlPanelConnector.h"

using namespace ControlPanelConnector;

class CalibrationSoftwareControlPanelConnector :
    public ControlPanelConnector
{

public:
	CalibrationSoftwareControlPanelConnector(std::string controlPanelIP,
		std::string eventPort,
		std::string myInterfaceIP,
		std::string statusPort);
	~CalibrationSoftwareControlPanelConnector();

	long EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0);
	bool GotStartCommand;
	bool GotStopCommand;
};

