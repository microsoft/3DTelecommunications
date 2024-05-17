#include <SDKDDKVer.h>

#include "network_loader.h"
#include "TimeUtil.h"

#include <omp.h>
#include <iostream>
#include <signal.h>

volatile int network_loader::g_RecvThreadRetVal = 0;

network_loader::network_loader(std::vector<std::string> hosts, std::vector<std::string> ports)
{
	// Connect to remote hosts
	asio::ip::tcp::resolver resolver(io_service_);
	system::error_code ec;
	for (int i = 0; i < DEPTH_CAMERAS_NUM; i++)
	{		
		auto endpoint = resolver.resolve({ hosts[i], ports[i] }, ec);
		if (ec) {
			LOGGER()->error("network_loader::network_loader", "Resolver %d (ec.value): (%d) %s", i, ec.value(), ec.message().c_str());
		}
		sockets_.push_back(new asio::ip::tcp::socket(io_service_));

		asio::connect(*sockets_[i], endpoint, ec);
		if (ec) {
			LOGGER()->error("network_loader::network_loader", "Connect %d (ec.value): (%d) %s", i, ec.value(), ec.message().c_str());
		}
		else {
			LOGGER()->warning("network_loader::network_loader", "%d connected to: %s:%d", i, sockets_[i]->remote_endpoint().address().to_string().c_str(), sockets_[i]->remote_endpoint().port());
		}
	}

	// investigate -- using page-locked (pinned) memory here?
	for (int i = 0; i < DEPTH_CAMERAS_NUM; i++) {
		buffer_.push_back(std::vector<char>((DEPTH_SIZE + COLOR_SIZE + 2 * IMAGE_HEADER_SIZE) * DEFAULT_PODS_PER_HOST + PACKET_HEADER_SIZE));
	}
}


network_loader::~network_loader()
{
	for (int i = 0; i < DEPTH_CAMERAS_NUM; i++)
		delete sockets_[i];
}


///todo: fix this, This is used when we're not using network_pinned memory
void network_loader::read_frame(int frame)
{
	int nBuffers = DEPTH_CAMERAS_NUM;

	std::vector<char*> dataBuffers(nBuffers);
	std::vector<size_t> dataSizes(nBuffers);
	for (auto i = 0; i < DEPTH_CAMERAS_NUM; i++) { dataBuffers[i] = &buffer_[i].front(); dataSizes[i] = (DEPTH_SIZE + COLOR_SIZE + 2 * IMAGE_HEADER_SIZE) * DEFAULT_PODS_PER_HOST + PACKET_HEADER_SIZE; }

}

void network_loader::read_frame_to(std::vector<int> &frames, int nBuffers, std::shared_ptr<AllTempCapturePackets> allCapPacket)
{
	g_RecvThreadRetVal = 0;
	auto t1 = prec_time::now();

#pragma omp parallel num_threads(DEPTH_CAMERAS_NUM) 
	{
		
		int i = omp_get_thread_num();
        network_loader::read_frame_from_network(&frames[i], sockets_[i], allCapPacket, i);
		
	}


	auto t2 = prec_time::now();
	auto time_ms = prec_time::as_ms(t1, t2);
	
	LOGGER()->trace(" --- network latency: %fms" , time_ms);

}

void network_loader::read_frame_from_network(int* frame, asio::ip::tcp::socket* socket, std::shared_ptr <AllTempCapturePackets> allCapPackets, int idx)
{
	size_t len, transfer_size;
	system::error_code ec;

	if (LOGGER()->check_verbosity(Logger::Trace)) {
		std::stringstream ss;
		ss << socket->local_endpoint().port() << " " << idx << " Requesting frame " << *frame << std::endl;
		LOGGER()->trace(ss.str().c_str());
	}

	len = asio::write(*socket, asio::buffer(frame, sizeof(int)), asio::transfer_at_least(sizeof(int)), ec);
	if (ec || len == 0) { 
		allCapPackets->currPacketVector[idx]->mFrameNumber = 0;
		allCapPackets->currPacketVector[idx]->mTimestamp = 0;
		return; }

	//TODO : time this out
	// Read header
	LOGGER()->trace("Reading header, waiting for 12 bytes...");

	len = asio::read(*socket, asio::buffer(allCapPackets->currPacketsDataPointers.packetHeader, PACKET_HEADER_SIZE), asio::transfer_exactly(PACKET_HEADER_SIZE), ec);
	
	LOGGER()->trace("Ok");

	unsigned long packetSize = reinterpret_cast<unsigned long*>(allCapPackets->currPacketsDataPointers.packetHeader)[0];
	unsigned long currFrameNum = reinterpret_cast<unsigned long*>(allCapPackets->currPacketsDataPointers.packetHeader)[1];
    unsigned long currTimestamp = reinterpret_cast<unsigned long*>(allCapPackets->currPacketsDataPointers.packetHeader)[2];
	
	if (ec || len == 0 || packetSize == 0) // total size is zero
	{
		allCapPackets->currPacketVector[idx]->mFrameNumber = 0;
		allCapPackets->currPacketVector[idx]->mTimestamp = 0;
		return;
	}
	allCapPackets->currPacketVector[idx]->mFrameNumber = currFrameNum;
	allCapPackets->currPacketVector[idx]->mTimestamp = currTimestamp;

	// Read depth
	for (int pod_index = 0; pod_index < DEFAULT_PODS_PER_HOST; pod_index++)
	{
		int camIndex = idx * DEFAULT_PODS_PER_HOST + pod_index;

		len = asio::read(*socket, asio::buffer(allCapPackets->currPacketsDataPointers.depth_imageHeaders[camIndex], IMAGE_HEADER_SIZE), asio::transfer_exactly(IMAGE_HEADER_SIZE), ec);
		if (ec || len == 0) { LOGGER()->error("network_loader::read_fram_from_network", "(ec.value) %d (%d) %s", ec.value(), 1, ec.message().c_str()); return; }
		transfer_size = *reinterpret_cast<int*>(allCapPackets->currPacketsDataPointers.depth_imageHeaders[camIndex]);
		len = asio::read(*socket, asio::buffer(allCapPackets->currPacketsDataPointers.depth_images[camIndex].data, DEPTH_SIZE), asio::transfer_exactly(transfer_size), ec);
		if (ec || len == 0) { LOGGER()->error("network_loader::read_fram_from_network", "(ec.value) %d (%d) %s", ec.value(), 2, ec.message().c_str()); return; }
	}

	// Read color
	for (int pod_index = 0; pod_index < DEFAULT_PODS_PER_HOST; pod_index++)
	{
		int camIndex = idx * DEFAULT_PODS_PER_HOST + pod_index;

		len = asio::read(*socket, asio::buffer(allCapPackets->currPacketsDataPointers.color_imageHeaders[camIndex], IMAGE_HEADER_SIZE), asio::transfer_exactly(IMAGE_HEADER_SIZE), ec);
		if (ec || len == 0) { LOGGER()->error("network_loader::read_fram_from_network","(ec.value) %d (%d) %s", ec.value(), 3,  ec.message().c_str()); return; }
		transfer_size = *reinterpret_cast<int*>(allCapPackets->currPacketsDataPointers.color_imageHeaders[camIndex]);
		len = asio::read(*socket, asio::buffer(allCapPackets->currPacketsDataPointers.color_images[camIndex].data, COLOR_SIZE), asio::transfer_exactly(transfer_size), ec);
		if (ec || len == 0) { LOGGER()->error("network_loader::read_fram_from_network","(ec.value) %d (%d) %s", ec.value(), 4,  ec.message().c_str()); return; }
	}


	// Read Audio
	if (data_format::AUDIO_SAMPLE_SIZE > 0 && data_format::AUDIO_SAMPLE_RATE > 0)
	{
		int audioOffset = PACKET_HEADER_SIZE + DEFAULT_PODS_PER_HOST * (IMAGE_HEADER_SIZE + DEPTH_SIZE) + DEFAULT_PODS_PER_HOST * (IMAGE_HEADER_SIZE + COLOR_SIZE);
		char* read_buffer = reinterpret_cast<char*> (allCapPackets->currPacketsDataPointers.audioSampleCountLocations[idx]);

		len = asio::read(*socket, asio::buffer(read_buffer, (size_t)data_format::AUDIO_SAMPLE_RATE * data_format::AUDIO_SAMPLE_SIZE + sizeof(int)), asio::transfer_exactly((size_t)data_format::AUDIO_SAMPLE_RATE * data_format::AUDIO_SAMPLE_SIZE + sizeof(int)), ec);

	}
	
	(*frame)++;//increment to indicate this frame was fetched

	if (ec)
		LOGGER()->error("network_loader::read_fram_from_network","(ec.value) %d (%d) %s", ec.value(), 5, ec.message().c_str());
	else
		g_RecvThreadRetVal = 1; // means we got some result
}

HRESULT network_loader::checkReceiveResult()
{
	if (g_RecvThreadRetVal > 0)
		return S_OK;
	return E_FAIL;
}