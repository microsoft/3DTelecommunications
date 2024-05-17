// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"
#include "ControlPanelConnectorDLL.h"

using namespace ControlPanelConnector;


#define DLLEXPORT __declspec( dllexport )

extern "C"
{
	//constructor/destructor
	DLLEXPORT void* create_control_panel_connector(const char* controlPanelIP,
		const char* eventTCPPort,
		const char* eventEPGMPort,
		const char* myInterfaceIP,
		const char* statusTCPPort,
		const char* statusEPGMPort);

	DLLEXPORT void destroy_control_panel_connector(void* pointer);

	//set callback function
	DLLEXPORT void connector_set_callback(void* pointer, EventReceivedCallback erc);
	
	DLLEXPORT long connector_send_status(void* pointer, CPC_STATUS status, void* statusData, int dataSize);

	DLLEXPORT void connector_set_build_version(void* pointer, const int major, const int minor, const int patch, const int commits, const char* branchName, const char* description, const char* sha);
}

void* create_control_panel_connector(const char* controlPanelIP,
	const char* eventTCPPort,
	const char* eventEPGMPort,
	const char* myInterfaceIP,
	const char* statusTCPPort,
	const char* statusEPGMPort)
{
	ControlPanelConnectorDLL* connector = new ControlPanelConnectorDLL(std::string(controlPanelIP),
		std::string(eventTCPPort),
		std::string(eventEPGMPort),
		std::string(myInterfaceIP),
		std::string(statusTCPPort),
		std::string(statusEPGMPort));
	return connector;
}

void destroy_control_panel_connector(void* pointer)
{
	ControlPanelConnectorDLL* client = reinterpret_cast<ControlPanelConnectorDLL*>(pointer);
	delete client;
	printf("deleted client");
}

long connector_send_status(void * pointer, CPC_STATUS status, void* statusData, int dataSize)
{
	ControlPanelConnectorDLL* connector = reinterpret_cast<ControlPanelConnectorDLL*>(pointer);
	//return client->SendStatusUpdate(vertex_data, index_data, color_images, audio_data);
	return connector->SendStatusUpdate(status, statusData, dataSize);
}

void connector_set_callback(void* pointer, EventReceivedCallback erc)
{
	ControlPanelConnectorDLL* connector = reinterpret_cast<ControlPanelConnectorDLL*>(pointer);
	connector->SetCallback(erc);
}

void connector_set_build_version(void* pointer, const int major, const int minor, const int patch, const int commits, const char* branchName, const char* description, const char* sha)
{
	ControlPanelConnectorDLL* connector = reinterpret_cast<ControlPanelConnectorDLL*>(pointer);
	connector->SetBuildVersion(major, minor, patch, commits, std::string(branchName), std::string(description), std::string(sha));
}

