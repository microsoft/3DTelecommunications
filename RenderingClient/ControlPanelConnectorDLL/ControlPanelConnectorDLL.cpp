#include "ControlPanelConnectorDLL.h"


ControlPanelConnectorDLL::ControlPanelConnectorDLL(std::string controlPanelIP,
	std::string eventTCPPort,
	std::string eventEPGMPort,
	std::string myInterfaceIP,
	std::string statusTCPPort,
	std::string statusEPGMPort) : ControlPanelConnector (controlPanelIP, eventTCPPort, myInterfaceIP, statusTCPPort, false)
{

}

long ControlPanelConnectorDLL::EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize)
{
   
    if (callback != NULL)
    {
        callback(eventType, eventData, dataSize);
    }
	return  ControlPanelConnector::EventReceived(eventType, eventData, dataSize);
}
void ControlPanelConnectorDLL::SetBuildVersion(const int major, const int minor, const int patch, const int commits, const std::string branchName, const std::string description, const std::string sha)
{
	ControlPanelConnector::SetVersionInformation(major, minor, patch, commits, branchName, description, sha);
}