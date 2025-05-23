#pragma once
// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "ControlPanelConnector.h"
using namespace ControlPanelConnector;

typedef long(__stdcall* EventReceivedCallback)(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize);

class ControlPanelConnectorDLL :
	public ControlPanelConnector
{
	
	EventReceivedCallback callback;

public:
	ControlPanelConnectorDLL(std::string controlPanelIP,
		std::string eventTCPPort,
		std::string eventEPGMPort,
		std::string myInterfaceIP,
		std::string statusTCPPort,
		std::string statusEPGMPort);

	void SetCallback(EventReceivedCallback erc)
	{
		callback = erc;
	}

	long EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData = NULL, int dataSize = 0);

	void SetBuildVersion(const int major, const int minor, const int patch, const int commits, const std::string branchName, const std::string description, const std::string sha);
};
