#pragma once

#include <string>

#include <thread>
#include <mutex>
#include <condition_variable>
#include "../Common/debug.h"
#include "Peabody.h"
#include "data_format.h"
#include "abstract_loader.h"
#include "DataCaptureThread.h"


class network_loader : public abstract_loader
{
public:
	network_loader(std::vector<std::string> hosts, std::vector<std::string> ports);
	~network_loader();

	void read_frame(int frame);
	//phildav:TODO better option - allocate buffer(pinned gpu or opencv) and pass those in to be filled by the receiving threads, since we are locked
	// to a format as it is.
	void read_frame_to(std::vector<int> &frame, int nBuffers, std::shared_ptr<AllTempCapturePackets> tempCapturePacketVector);

	int frame_count() const { return 0; }
	int cached_frame_count() const { return 0; }

	const std::vector<char>& get_buffer_for_pod(int i) const { return buffer_[i]; }

	static HRESULT checkReceiveResult();
	//
	//  format of packet 
	///      packetheader   |        pod 1                      pod2          |        pod1                        pod2   
	//     dword     dword  |   dword     depthsz1       dword      depthsz2  |  dword     colorsz1        dword       colorsz2
	//  [pcktsize][framenum]|[depthsz1][depthdat... ]|[depthsz2][depthdat2...]|[colorsz1][colordat... ]|[colorsz2...][colordat2... ]
	//                      |                        |                        |
	
	const char* get_depth_image_data(int i) const 
	{
		int pod = i / DEFAULT_PODS_PER_HOST;
		int pod_cam_index = i % DEFAULT_PODS_PER_HOST;

		auto& pod_buffer = get_buffer_for_pod(pod);
		if (pod_buffer.empty()) return 0;

        auto depth_total_size = IMAGE_HEADER_SIZE + data_format::DEPTH_HEIGHT * data_format::DEPTH_WIDTH * data_format::DEPTH_BYTES_PER_PIXEL;
		auto depth_data_offset_i = PACKET_HEADER_SIZE + (pod_cam_index * depth_total_size) + IMAGE_HEADER_SIZE;
		return &pod_buffer[depth_data_offset_i];
	}

	const char* get_color_image_data(int i) const 
	{
		int pod = i / DEFAULT_PODS_PER_HOST;
		int pod_cam_index = i % DEFAULT_PODS_PER_HOST;

		auto& pod_buffer = get_buffer_for_pod(pod);
		if (pod_buffer.empty()) return 0;

        auto depth_imagery_size = DEFAULT_PODS_PER_HOST * (IMAGE_HEADER_SIZE + data_format::DEPTH_WIDTH * data_format::DEPTH_HEIGHT * data_format::DEPTH_BYTES_PER_PIXEL);

        auto color_image_size = IMAGE_HEADER_SIZE + data_format::COLOR_WIDTH * data_format::COLOR_HEIGHT * data_format::COLOR_BYTES_PER_PIXEL;
		auto color_data_offset_i = PACKET_HEADER_SIZE + depth_imagery_size + (pod_cam_index * color_image_size) + IMAGE_HEADER_SIZE;
		return &pod_buffer[color_data_offset_i];
	}

private:
	static void read_frame_from_network(int* frame, asio::ip::tcp::socket* socket, std::shared_ptr<AllTempCapturePackets> allCapPackets, int idx);



	volatile static int g_RecvThreadRetVal;

	std::vector<std::vector<char>> buffer_;

	asio::io_service io_service_;
	std::vector<asio::ip::tcp::socket*> sockets_;
};
