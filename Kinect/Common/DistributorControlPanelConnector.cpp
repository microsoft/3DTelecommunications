#include "DistributorControlPanelConnector.h"


DistributorControlPanelConnector::DistributorControlPanelConnector()
{
	GotStopCommand = false;
	GotStartCommand = false;
	EncoderIP = ENCODER_SERVER_IP;
}


DistributorControlPanelConnector::~DistributorControlPanelConnector()
{
	// should be a try, could fail if I never got a ready notification
	try
	{
		delete EncoderIP;
	}
	catch (std::exception)
	{

	}
}

HRESULT DistributorControlPanelConnector::EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize)
{
	ControlPanelConnector::EventReceived(eventType, eventData, dataSize);
	switch (eventType)
	{
	case ENCODER_READY:
		if (dataSize > 0)
		{
			// parse out the IP address
			EncoderIP = (char*)malloc(dataSize);
			memcpy_s(EncoderIP, dataSize, ((char*)eventData) + 1, dataSize); // +1 to skip the eventType bit
			EncoderIP[dataSize - 1] = '\0'; // zero mq doesn't send a \0 with strings, so we have to add it
		}
		break;
	case ENCODER_FPS:
		// Do nothing
		printf("Got encoder FPS: %f\n", *((float*)eventData));
		break;
	case DISTRIBUTOR_FPS:
		break;
	case DEPTH_GEN_FPS:
		printf("Got depth gen FPS: %f\n", *((float*)eventData));
		break;
	case FUSION_FPS:
		printf("Got fusion FPS: %f\n", *((float*)eventData));
		break;
	case RENDER_FPS:
		printf("Got render FPS: %f\n", *((float*)eventData));
		break;

	case DISTRIBUTOR_SLOW_DOWN_REQUEST:
		break;
	case DEPTH_GEN_SLOW_DOWN_REQUEST:
		printf("Depth gen requesting slowdown.\n");
		break;
	case FUSION_SLOW_DOWN_REQUEST:
		printf("Fusion requesting slowdown.  Prepare for possible depth gen slowdown request\n");
		break;
	case RENDER_SLOW_DOWN_REQUEST:
		// do nothing
		break;

	case CONTROL_PANEL_START_REQUESTED:
		// I don't actually use this value.  The distributor just waits until it has an encoder to connect to and then starts listening.  Depth gen clients can technically connect at any time
		GotStartCommand = true;
		printf("Got a system start request!\n");
		break;
	case CONTROL_PANEL_STOP_REQUESTED:
		GotStopCommand = true;
		printf("Got a system stop request!\n");
		break;
	default:
		break;
	}

	return S_OK;
}
