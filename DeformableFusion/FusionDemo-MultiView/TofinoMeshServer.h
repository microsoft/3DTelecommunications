#pragma once 

#include "data_format.h"
#include "TofinoPacket.h"
#include "DataCaptureThread.h"

#include "fusion_srv\ICompressor.h"
#include "fusion_srv\ConnectionManager.h"

#include <condition_variable>
#include <mutex>
#include <deque>

#include "utility.h"

#define TOFINO_MESH_SERVER_PORT 14900
#define MESH_SERVER_FRAME_NUM 10

namespace tfutil {
	template<class T> 
	const T* vec_data_ptr(const std::vector<T>& vec) { return (vec.empty()) ? nullptr : &vec.front(); }

	template<class T>
	T* vec_data_ptr(std::vector<T>& vec) { return (vec.empty()) ? nullptr : &vec.front(); }
};

struct TofinoMeshPayloadV1 
{
	MeshHeaderVersion header;

	prec_time::timepoint start_time;
	prec_time::timepoint buffer_request_time;
	prec_time::timepoint dispatch_time;

	std::vector<char>	raw_vertex_index_data;
	std::vector<char>	raw_color_payload;
	std::vector<char>	mesh_packet_out; //allocate full size
};


class TofinoMeshServer
{
public:
	enum MeshFrameState
	{
		MeshFrame_Ready =0, //ready for next incoming 
		MeshFrame_Staging = 1, //gathering data 
		MeshFrame_Dispatched = 2, //data ready for prep thread
		MeshFrame_Preparing =3, //preparing packet for dispatch
	};

public:
	TofinoMeshServer()
	{
		for (auto& v : m_mesh_payload) { 
			v.mesh_packet_out.reserve(40 * 1024 * 1024); 
			v.header = {};
		}
	}

	int get_current_frame() const { return m_frame_count; }
	int get_current_connection_count();

	static int  get_pinned_buffer_count_needed() { return MESH_SERVER_FRAME_NUM; }
	void set_pinned_buffer_pointers(char** pinned_vertex_buffers, char** pinned_index_buffers);
	void set_run_asynchronous(bool b) { m_prepare_asynchronous = b; }
	void server_mainloop();

	void initialize_server(unsigned short port = TOFINO_MESH_SERVER_PORT);
	void initialize_mock_server(const char * filepattern, int start, int end, unsigned short port = TOFINO_MESH_SERVER_PORT);

	bool	is_next_frame_ready();
	int		start_next_frame(int frame_number);

	void	set_frame_color_data(int id, char**data, std::vector<int> colorSizeToCopy, int numimages);
	void    copy_frame_color_data(int id);

	void    set_frame_audio_data(int id, std::vector<char*> audioData, std::vector<int> audioDataSize, int numDataBuffers);
	void    copy_frame_audio_data(int id);

	void	set_frame_geometry_counts(int id, unsigned int nverts, unsigned int ntris);
	void	get_frame_data_ptrs(int id, char** vtx_dest, char** idx_dest);
	bool	notify_frame_data_ptrs_ready(int id, char* vtx, char* idx);
	void	dispatch_frame(int id);

	void	prepare_outgoing_packet(int id);

	void	stop_server();

	void	set_log_file_level(int level) { m_log_to_file_level = level; }
	void	set_log_console_level(int level) { m_log_to_console_level = level; }
	void	set_file_capture_location(const std::string &loc) { m_packet_to_file_output_location = loc; }

	bool	is_capturing_to_file() const { return !m_packet_to_file_output_location.empty(); }
	bool	is_logging_to_file() const { return m_log_to_file_level != 0; }
private:
	void update_frame_stats(prec_time::timepoint t_start, int frame, int frame_bytes, int frame_verts, prec_time::timepoint t_prep, prec_time::timepoint t_end);

	class FileFrameMockSource;
	class I3DCompressorAdaptor;

	std::shared_ptr<ICompressor> m_frame_source;
	std::shared_ptr<CConnectionManager> m_server;

	int	get_latest_prepared_frame(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber);

	std::mutex						m_frame_buffer_mutex;
	std::mutex						m_colorCopyMutex;

	std::map<int, int>				m_frame_to_staging_map;

	int								m_frame_state[MESH_SERVER_FRAME_NUM];
	char*							m_pinned_vtx_buffer[MESH_SERVER_FRAME_NUM]; //receive from marching_cubes
	char*							m_pinned_ind_buffer[MESH_SERVER_FRAME_NUM]; //receive from marching_cubes
	unsigned int					m_vertex_count[MESH_SERVER_FRAME_NUM];
	unsigned int					m_tri_count[MESH_SERVER_FRAME_NUM];
	std::vector<char*>				m_pinned_color_buffers[MESH_SERVER_FRAME_NUM];
	std::vector<char*>		        m_pinned_audio_buffers[MESH_SERVER_FRAME_NUM];
	std::vector<int>				m_pinned_color_buffer_sizes[MESH_SERVER_FRAME_NUM];
	std::vector<int>				m_pinned_audio_buffer_sizes[MESH_SERVER_FRAME_NUM];
	TofinoMeshPayloadV1				m_mesh_payload[MESH_SERVER_FRAME_NUM];

	int								m_current_frame = -1;
	int								m_staging_frame = -1;

	int								m_preparing_frame = -1;

	bool							m_prepare_asynchronous = true;
	std::mutex						m_prep_mutex;
	std::condition_variable			m_prep_cv;

	int								m_frame_count = 0;
	bool							m_process_loop_active = true;

	int								m_log_to_file_level = 0;
	int								m_log_to_console_level = LOGGER()->get_verbosity();
	std::string						m_packet_to_file_output_location;

	typedef  std::tuple<prec_time::timepoint, int, int, int, prec_time::timepoint, prec_time::timepoint> frame_stat_tuple;
	// start fnum fbytes fverts

public:
	bool							m_show_stats = false;
	std::deque<frame_stat_tuple>	m_fstats;
	double							m_gbps = 0;
	double							m_fps =0;
	double							m_vtxps = 0;
	double							m_avg_prep_ms = 0;
	double							m_avg_latency_ms = 0;

private:
	bool							m_in_mock_mode = false;
};

class TofinoMeshServer::I3DCompressorAdaptor 
	: public ICompressor
{
public:
	I3DCompressorAdaptor(TofinoMeshServer* server) : m_server(server) {}
	virtual HRESULT GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber) override
	{
		auto res = m_server->get_latest_prepared_frame(ppData, pcbData, pdFrameNumber);
		return (res == 0) ? S_OK : E_FAIL;
	}

private:
	TofinoMeshServer* m_server;
};

