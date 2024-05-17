#include "TofinoMeshServer.h"
#include "data_format.h"
#include "lz4.h"
#include "network_loader.h"

namespace {
	void set_current_format_version(TofinoPacketHeader& phead){
		memcpy(phead.frame_format, "CMSH", 4);
		phead.frame_format_ver = 100;
	}

}

class TofinoMeshServer::FileFrameMockSource
	: public ICompressor
{
public:
	FileFrameMockSource(const char * pattern, int start_idx, int end_idx);
	virtual HRESULT GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber) override;

private:
	void					preload_all_frames();
	std::vector<std::vector<char>>		m_frames;

	prec_time::timepoint			m_start_time;
	int								m_frame_num;

	std::string						m_mock_pattern;
	int								m_mock_idx_start;
	int								m_mock_idx_end;

};


void TofinoMeshServer::set_pinned_buffer_pointers(char** pinned_vtx_buffers, char** pinned_ind_buffers)
{
	auto n_buffers = get_pinned_buffer_count_needed();

	for (int i = 0; i < n_buffers; ++i) {
		m_pinned_vtx_buffer[i] = pinned_vtx_buffers[i]; //int count, then data
		m_pinned_ind_buffer[i] = pinned_ind_buffers[i]; //int count, then data
	}
}

void TofinoMeshServer::update_frame_stats(
	prec_time::timepoint t_start, 
	int frame_num, 
	int frame_bytes, 
	int frame_verts, 
	prec_time::timepoint t_prep, 
	prec_time::timepoint t_end)
{
	m_fstats.push_back(frame_stat_tuple(t_start, frame_num, frame_bytes, frame_verts, t_prep, t_end));
	while (m_fstats.size() > 60) { m_fstats.pop_front(); }


	if (m_fstats.size() > 1)
	{
		double	window_sec = prec_time::as_sec(std::get<0>(m_fstats.front()), std::get<0>(m_fstats.back()));
		int		num_frames = m_fstats.size();
		double	frame_div = 1.0 / num_frames;

		int		total_bytes = 0;
		int		total_verts = 0;
		double	total_prep = 0;
		double	total_latency = 0;

		for (auto f : m_fstats) {
			total_bytes += get<2>(f);
			total_verts += get<3>(f);
			total_prep += prec_time::as_ms(get<4>(f), get<5>(f));
			total_latency += prec_time::as_ms(get<0>(f), get<5>(f));
		}

		m_fps = num_frames / window_sec;
		m_gbps = (1.e-9 * total_bytes * CHAR_BIT) / window_sec;

		m_vtxps = total_verts * frame_div;
		m_avg_prep_ms = total_prep * frame_div;
		m_avg_latency_ms = total_latency * frame_div;

	}
}


void TofinoMeshServer::initialize_server(unsigned short port)
{	
	for (int i = 0; i < MESH_SERVER_FRAME_NUM; ++i) {
		m_frame_state[i] = MeshFrame_Ready;
	}

	m_frame_source = std::make_shared<I3DCompressorAdaptor>(this);

	m_server = std::make_shared<CConnectionManager>();
	m_server->SetInput(m_frame_source.get());
	m_server->Startup(port);
}

void TofinoMeshServer::initialize_mock_server(const char* filepattern, int start, int end, unsigned short port)
{
	m_in_mock_mode = true;
	m_frame_source = std::make_shared<FileFrameMockSource>(filepattern, start, end);
	m_server = std::make_shared<CConnectionManager>();
	m_server->SetInput(m_frame_source.get());
	m_server->Startup(port);
}

void TofinoMeshServer::stop_server()
{
	m_server->Shutdown();
}


namespace
{
	int next_mesh_buffer(int cur) { return (cur + 1) % MESH_SERVER_FRAME_NUM; }
}

void TofinoMeshServer::server_mainloop()
{
	if (m_in_mock_mode)
	{
		while (m_prepare_asynchronous || m_process_loop_active)
		{
			Sleep(1000);
		}
	}

	if (m_prepare_asynchronous)
	{
		//asynchrony
		auto pred = [this]()
		{
			return !m_process_loop_active || (m_preparing_frame != -1 && m_frame_state[m_preparing_frame] == MeshFrame_Dispatched);
		};

		while (m_process_loop_active)
		{
			
			{
				std::unique_lock<std::mutex> tlock(m_prep_mutex);
				m_prep_cv.wait(tlock, pred);
				if (!m_process_loop_active) { break; }

				assert(m_frame_state[m_preparing_frame] == MeshFrame_Dispatched); // we move dispatched->preparing

				m_frame_state[m_preparing_frame] = MeshFrame_Preparing;
				prepare_outgoing_packet(m_preparing_frame);
			}

			

			m_frame_state[m_preparing_frame] = MeshFrame_Ready;

			m_current_frame = m_preparing_frame; //last frame prepared


			auto& current_frame = m_mesh_payload[m_current_frame];
			m_preparing_frame = next_mesh_buffer(m_preparing_frame);
		

		}
		
	}
	else
	{
		while (m_process_loop_active) { Sleep(1000); }
	}
	
	stop_server();
}

int	TofinoMeshServer::get_latest_prepared_frame(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber)
{

	auto& current_frame = m_mesh_payload[m_current_frame];
	*pcbData = current_frame.mesh_packet_out.size();
	*ppData = reinterpret_cast<BYTE*>(&current_frame.mesh_packet_out.front());
	*pdFrameNumber = current_frame.header.packet_header.frame_number;

	return S_OK;
}

bool TofinoMeshServer::is_next_frame_ready()
{
	int potentialNextFrame = (m_staging_frame + 1) % MESH_SERVER_FRAME_NUM;;
	return potentialNextFrame != m_current_frame;
};

int TofinoMeshServer::start_next_frame(int frame_number)
{
	std::lock_guard<std::mutex> sl(m_frame_buffer_mutex); //get next staging frame

	m_frame_count = frame_number;
	//move staging_frame index forward
	m_staging_frame = (m_staging_frame + 1) % MESH_SERVER_FRAME_NUM;

	if (m_staging_frame == m_current_frame)
	{
		*LOGGER() << Logger::Warning << "WARNING: preparing much slower than fusion." << Logger::endl;
	}
	m_frame_state[m_staging_frame] = MeshFrame_Staging; //update frame state to "gathering data"
	m_frame_to_staging_map[frame_number] = m_staging_frame;
	m_mesh_payload[m_staging_frame].start_time = prec_time::now();
	m_mesh_payload[m_staging_frame].header.packet_header.frame_number = frame_number; //set the correct frame number here based on fusion's frameCount
	return m_staging_frame;
}

void TofinoMeshServer::dispatch_frame(int frame_number)
{
	int buffer_id = -1;
	{
		std::lock_guard<std::mutex> state_lock(m_frame_buffer_mutex);
		auto it = m_frame_to_staging_map.find(frame_number);
		if (it != m_frame_to_staging_map.end())
		{
			buffer_id = it->second;
		}
		else { assert(0); }
	}

	if (!m_prepare_asynchronous)
	{
		prepare_outgoing_packet(m_preparing_frame);

		m_current_frame = m_preparing_frame;
	}
	else
	{
		if (buffer_id != -1)
		{
			{
				std::lock_guard<std::mutex> prep_lock(m_prep_mutex);
			
				if (m_preparing_frame == -1) { m_preparing_frame = buffer_id; } // after this, the prepare thread advances
				m_mesh_payload[buffer_id].dispatch_time = prec_time::now();
				m_frame_state[buffer_id] = MeshFrame_Dispatched;
			}
			m_prep_cv.notify_one(); //thread should check that there isn't a frame waiting for prep
		}
	}
}

namespace
{
	FILE* logpackout = nullptr;
}

void TofinoMeshServer::set_frame_audio_data(int id, std::vector<char*> audioData, std::vector<int> audioDataSize, int numDataBuffers)
{
	m_pinned_audio_buffers[id].resize(numDataBuffers);
	m_pinned_audio_buffer_sizes[id].resize(numDataBuffers);
	for (int i = 0; i < numDataBuffers; ++i)
	{
		m_pinned_audio_buffers[id][i] = audioData[i];
		m_pinned_audio_buffer_sizes[id][i] = audioDataSize[i];
	}
}

void TofinoMeshServer::copy_frame_audio_data(int id)
{
	//set_frame_audio_data  + copy_frame_color_data BOTH must have been called before we get here

	auto& out = m_mesh_payload[id];
	auto num_audio_streams = m_pinned_audio_buffers->size();

	out.header.audio_data_count = num_audio_streams;
	out.header.audio_data_size = 0;

	if (data_format::AUDIO_SAMPLE_RATE <= 0 ||  data_format::AUDIO_SAMPLE_SIZE <= 0 )
	{
		return;
	}

	for (int i = 0; i < num_audio_streams; ++i)
	{
		out.header.audio_data_size += data_format::AUDIO_SAMPLE_RATE * data_format::AUDIO_SAMPLE_SIZE;
	}
	out.header.audio_data_size += num_audio_streams * sizeof(int); // one for each stream to inform size


	out.mesh_packet_out.resize(sizeof(MeshHeaderVersion) + out.header.color_data_size + out.header.audio_data_size + out.header.vertex_data_size + out.header.tri_data_size); //conservative

	char* base_ptr = &out.mesh_packet_out.front();
	char* data_region = base_ptr + sizeof(MeshHeaderVersion) + out.header.color_data_size;

	//copy in each of the audio stream size
	auto audio_size_ptr = tfutil::vec_data_ptr(m_pinned_audio_buffer_sizes[id]);
	int audio_sub_offset = sizeof(int) * num_audio_streams;
	memcpy(data_region, audio_size_ptr, audio_sub_offset);
	
	//copy actual audio data
	
	for (int i = 0; i < num_audio_streams; ++i) 
	{
		memcpy(data_region + audio_sub_offset, m_pinned_audio_buffers[id][i], data_format::AUDIO_SAMPLE_RATE * data_format::AUDIO_SAMPLE_SIZE);
		audio_sub_offset += data_format::AUDIO_SAMPLE_RATE * data_format::AUDIO_SAMPLE_SIZE;
	}
}


void TofinoMeshServer::set_frame_color_data(int id, char** data, std::vector<int> colorSizeToCopy, int numimages)
{
	std::lock_guard<std::mutex> lock(m_colorCopyMutex);
	m_pinned_color_buffers[id].resize(numimages);
	m_pinned_color_buffer_sizes[id].resize(numimages);
	for (int i = 0; i < numimages; i++)
	{
		m_pinned_color_buffers[id][i] = data[i];
		m_pinned_color_buffer_sizes[id][i] = colorSizeToCopy[i];
	}
}


void TofinoMeshServer::copy_frame_color_data(int id)
{
	//setframecolordata must have been called before we get here
	std::lock_guard<std::mutex> lock(m_colorCopyMutex);
	auto& out = m_mesh_payload[id];

	// get sizes
	auto& pinned_color = m_pinned_color_buffers[id];
	auto num_color_images = pinned_color.size();

	std::vector<unsigned int> compressed_color_sizes(num_color_images);

	int total_color_data_size = 0;

	for (int i = 0; i < pinned_color.size(); ++i) 
	{			
		int colorSize = m_pinned_color_buffer_sizes[id][i];

		bool isNetworkCapture = FusionConfig::current()->GetValueWithDefault<bool>("Fusion", "livecapture", true);;
		
		compressed_color_sizes[i] = colorSize;
		total_color_data_size += colorSize; 
	}

	total_color_data_size += num_color_images * sizeof(int); //write an array of sizes prior to packed compressed color data

	out.header.color_data_count = num_color_images;
	out.header.color_data_format = data_format::COLOR_FORMAT;
	
	out.header.color_data_width = data_format::COLOR_WIDTH;
	out.header.color_data_height = data_format::COLOR_HEIGHT;
	out.header.color_data_size = total_color_data_size;

	out.mesh_packet_out.resize(sizeof(MeshHeaderVersion) + out.header.color_data_size + out.header.audio_data_size + out.header.vertex_data_size + out.header.tri_data_size); //conservative


	char* base_ptr = &out.mesh_packet_out.front();
	char* data_region = base_ptr + sizeof(MeshHeaderVersion);

	char* color_sizes_offset = reinterpret_cast<char*>(data_region);
	char* color_data_offset = data_region + pinned_color.size() * sizeof(int);

	// write color sizes
	if (compressed_color_sizes.size()){
		auto color_size_ptr = tfutil::vec_data_ptr(compressed_color_sizes);
		memcpy(color_sizes_offset, color_size_ptr, sizeof(unsigned int)* num_color_images);
	}
	// write color data // TODO: parallel
	for (int i = 0; i < num_color_images; ++i) {
		//schedule all N
		memcpy(color_data_offset, pinned_color[i], compressed_color_sizes[i]);
		color_data_offset += compressed_color_sizes[i];
	}

}

int TofinoMeshServer::get_current_connection_count()
{
	return m_server->GetNumConnections(); //global, but ok for now
}

void TofinoMeshServer::prepare_outgoing_packet(int id)
{
	//track time
	static auto epoch = prec_time::now();

	//compression settings

	bool compress_vertex_table = true;
	bool compress_index_table = true;


	auto t0 = prec_time::now();

	auto tc1 = prec_time::now();

	if (!m_prepare_asynchronous) //if we're fully synchronous, we copy color here.
	{
		copy_frame_color_data(id);
	}

	auto tc2 = prec_time::now();

	// voila!
	auto& out = m_mesh_payload[id];

	// set header data

	set_current_format_version(out.header.packet_header);


	unsigned int total_size = sizeof(MeshHeaderVersion);
	// color imagery



	int total_data_offset_size = out.header.color_data_size + out.header.audio_data_size;
	auto t1 = prec_time::now();

	auto vertexFormatFlag = TofinoFormat::FORMAT_HALF_FLOAT6_P_N  | (compress_vertex_table ? TofinoCompress::LZ4 : TofinoCompress::RAW_DATA);

	bool isHQ = out.header.packet_header.frame_number == GlobalDataStatic::HighQualityFrameNumber - GlobalDataStatic::FrameStartEndOffset;
	if (isHQ)
	{
		vertexFormatFlag = vertexFormatFlag | TofinoCompress::HIGHQUALITY;
	}
	out.header.vertex_count = m_vertex_count[id];
	out.header.vertex_data_format = vertexFormatFlag; 
	auto vertElemSize = TofinoFormat::GetSize(out.header.vertex_data_format);
	out.header.vertex_full_size_bytes = out.header.vertex_count * vertElemSize;
	out.header.vertex_data_size = out.header.vertex_count * vertElemSize;

	out.header.tri_count = m_tri_count[id];
	out.header.tri_data_format = TofinoFormat::FORMAT_INT3 | (compress_index_table ? TofinoCompress::LZ4 : TofinoCompress::RAW_DATA);
	auto triElemSize = TofinoFormat::GetSize(out.header.tri_data_format);
	out.header.tri_full_size_bytes = out.header.tri_count * triElemSize;
	out.header.tri_data_size = out.header.tri_full_size_bytes;

	out.mesh_packet_out.resize(sizeof(MeshHeaderVersion) + out.header.color_data_size + out.header.audio_data_size + out.header.vertex_data_size + out.header.tri_data_size); //conservative

	auto t2 = prec_time::now();

	char* base_ptr = &out.mesh_packet_out.front();
	char* data_region = base_ptr + sizeof(MeshHeaderVersion);

	char* vertex_data_offset = data_region + total_data_offset_size;
	char* tri_data_offset = data_region + total_data_offset_size + out.header.vertex_full_size_bytes; //only if raw

	// color cpy
	auto t3 = prec_time::now();

	//
	// vertex table
	//

	if (compress_vertex_table) {
		// compress index data into buffer
		auto vert_compressed_size = LZ4_compress_fast(
			m_pinned_vtx_buffer[id] + sizeof(int),
			vertex_data_offset,
			out.header.vertex_full_size_bytes,
			out.header.vertex_full_size_bytes,
			data_format::LZ4_COMPRESS_FAST_LEVEL);

		// update header
		if (vert_compressed_size > 0)
		{
			out.header.vertex_data_size = vert_compressed_size; //update after compression
			tri_data_offset = vertex_data_offset + vert_compressed_size; // moves index packing location
		}
		else
		{
			//compress failed! copy in the original size			
			out.header.vertex_data_format = TofinoFormat::GetFormat(out.header.vertex_data_format) | TofinoCompress::RAW_DATA;
			if (isHQ)
			{
				out.header.vertex_data_format = out.header.vertex_data_format | TofinoCompress::HIGHQUALITY;
			}
			memcpy(vertex_data_offset, m_pinned_vtx_buffer[id] + sizeof(int), out.header.vertex_full_size_bytes);
		}
	}
	else {
		// copy vertex data to buffer
		memcpy(vertex_data_offset, m_pinned_vtx_buffer[id] + sizeof(int), out.header.vertex_full_size_bytes);
	}

	auto t4 = prec_time::now();
	//
	// triangle index table
	//

	if (compress_index_table)
	{
		// compress index data into buffer
		auto tri_compressed_size = LZ4_compress_fast(
			m_pinned_ind_buffer[id] + sizeof(int),
			tri_data_offset,
			out.header.tri_full_size_bytes,
			out.header.tri_full_size_bytes,
			data_format::LZ4_COMPRESS_FAST_LEVEL);

		// TODO -- allow this call to fail.
		if (tri_compressed_size > 0)
		{
			// update header
			out.header.tri_data_size = tri_compressed_size; //update after compression
		}
		else
		{
			//set format indicator to uncompressed, fall back to regular memcpy
			out.header.tri_data_format = TofinoFormat::GetFormat(out.header.tri_data_format) | TofinoCompress::RAW_DATA;
			memcpy(tri_data_offset, m_pinned_ind_buffer[id] + sizeof(int), out.header.tri_full_size_bytes);
		}

	}
	else {
		memcpy(tri_data_offset, m_pinned_ind_buffer[id] + sizeof(int), out.header.tri_full_size_bytes);
	}

	auto t5 = prec_time::now();

	out.header.packet_header.total_size = sizeof(MeshHeaderVersion)+out.header.color_data_size + out.header.audio_data_size+ out.header.vertex_data_size + out.header.tri_data_size;
	memcpy(base_ptr, &out.header, sizeof(MeshHeaderVersion));
	// snap (resize will not reallocate on shrink);
	out.mesh_packet_out.resize(out.header.packet_header.total_size); //conservative
	// write header

	auto t6 = prec_time::now();

	if (LOGGER()->check_verbosity(Logger::Trace) || m_log_to_file_level)
	{
		auto &oh = out.header;

		auto printstats = [&](FILE* fp, char sep, int level)
		{
			auto fnum = out.header.packet_header.frame_number;

			auto index_ratio = 100.0 * (oh.tri_data_size/ (1.0 * oh.tri_full_size_bytes));
			auto vertex_ratio = 100.0 * (oh.vertex_data_size / (1.0 * oh.vertex_full_size_bytes));
			auto color_ratio = 100.0 * (oh.color_data_size/ (1.0 * oh.color_data_height * oh.color_data_width * CHAR_BIT));


			fprintf(fp, "frame %d %lf %lf buffer %d verts %d trias %d color %d%c", fnum, prec_time::as_sec(epoch, t0), prec_time::as_sec(t0, t6), id, oh.vertex_count, oh.tri_count, oh.color_data_count, sep);
			fprintf(fp, "sizes %d total %d vsize %d tsize %d csize %d%c", fnum, static_cast<int>(out.mesh_packet_out.size()), oh.vertex_data_size, oh.tri_data_size, oh.color_data_size, sep);

			if (level >= 3)
			{
				fprintf(fp, "ratio %d tri:%lf vertex:%lf color:%lf %c", fnum, index_ratio, vertex_ratio, color_ratio, sep);
				fprintf(fp, "times %d prep:%lf s:%lf c:%lf r:%lf v:%lf i:%lf f:%lf s_to_req:%lf s_to_disp:%lf s_to_prep:%lf total:%lf %c", fnum,
					prec_time::as_ms(t0, t6),
					prec_time::as_ms(t0, t1), prec_time::as_ms(t1, t2), prec_time::as_ms(t2, t3),
					prec_time::as_ms(t3, t4), prec_time::as_ms(t4, t5), prec_time::as_ms(t5, t6),
					prec_time::as_ms(out.start_time, out.buffer_request_time), //request buffers
					prec_time::as_ms(out.start_time, out.dispatch_time), //buffer data complete
					prec_time::as_ms(out.start_time, t0), //prep started
					prec_time::as_ms(out.start_time, t6),
					sep);
			int* triangle_start = reinterpret_cast<int*>(m_pinned_ind_buffer[id] + 4);

			fprintf(fp, "start-tri %d: %d %d %d%c", fnum, triangle_start[0], triangle_start[1], triangle_start[2], sep);

			short* vert_start = reinterpret_cast<short*>(m_pinned_vtx_buffer[id] + 4);
			fprintf(fp, "start-vert %d: %hd %hd %hd%c", fnum, vert_start[0], vert_start[1], vert_start[2], sep);
			}
			fprintf(fp, "\n");
		};

		if (LOGGER()->check_verbosity(Logger::Trace)) { printstats(stderr, '\n', m_log_to_console_level); }

		if (m_log_to_file_level)
		{
			if (logpackout == nullptr){ logpackout = fopen(".//packet_logs.txt", "a"); }
			if (logpackout != nullptr)
			{
				printstats(logpackout, ' ', m_log_to_file_level);
				fflush(logpackout);
			}
		}
	}


	if (!m_packet_to_file_output_location.empty())
	{
		char name[200];
		sprintf(name, m_packet_to_file_output_location.c_str(), out.header.packet_header.frame_number);
		std::ofstream file_out(name, std::ios::binary);
		file_out.write(&out.mesh_packet_out.front(), out.mesh_packet_out.size());
		file_out.close();
	}

	if (m_show_stats)
	{		
		update_frame_stats(out.start_time, m_frame_count, out.header.packet_header.total_size, out.header.vertex_count, t0, t6);
	}
}


void TofinoMeshServer::set_frame_geometry_counts(int id, unsigned int nverts, unsigned int ntris)
{
	m_vertex_count[id] = nverts;
	m_tri_count[id] = ntris;
}

void TofinoMeshServer::get_frame_data_ptrs(int id, char** vtx_buffer, char** ind_buffer)
{
	*vtx_buffer = m_pinned_vtx_buffer[id];
	*ind_buffer = m_pinned_ind_buffer[id];
	m_mesh_payload[id].buffer_request_time = prec_time::now();
}

bool TofinoMeshServer::notify_frame_data_ptrs_ready(int id, char* vtx_buf, char* ind_buf)
{
	if (m_pinned_vtx_buffer[id] == vtx_buf && m_pinned_ind_buffer[id] == ind_buf) 
	{
		m_vertex_count[id] = reinterpret_cast<int*>(vtx_buf)[0];
		m_tri_count[id] = reinterpret_cast<int*>(ind_buf)[0];
		return true;
	}

	LOGGER()->error("TofinoMeshServer::notify_frame_data_ptrs_ready", "notify -- pointers didnt' match\n");
	return false;
}


TofinoMeshServer::FileFrameMockSource::FileFrameMockSource(const char* pattern, int start_idx, int end_idx)
	: m_mock_pattern(pattern)
	, m_mock_idx_start(start_idx)
	, m_mock_idx_end(end_idx)
	, m_frame_num(0)
{
	m_start_time = prec_time::now();
	preload_all_frames();
}

void TofinoMeshServer::FileFrameMockSource::preload_all_frames()
{
	char name[512];
	for (auto i = m_mock_idx_start; i <= m_mock_idx_end; ++i)
	{
		sprintf(name, m_mock_pattern.c_str(), i);
		ifstream in(name, ios::binary);

		if (in.good())
		{
			in.seekg(0, ios::end);
			auto len = in.tellg();
			in.seekg(0, ios::beg);

			if (len.seekpos() == 0) { LOGGER()->error("TofinoMeshServer::FileFrameMockSource::preload_all_frames", "empty file %s\n", name); }
			std::vector<char> readbuf(len);
			in.read(&readbuf.front(), len);

			if (len.seekpos() == 4) { LOGGER()->error("TofinoMeshServer::FileFrameMockSource::preload_all_frames", "empty packet? %s\n", name); }
			if (len.seekpos() < 50) { LOGGER()->error("TofinoMeshServer::FileFrameMockSource::preload_all_frames", "not enough data %s\n", name); }

			int* h_info = reinterpret_cast<int*>(&readbuf.front());
			if (h_info[0] != len)
			{
				if (len.seekpos() < 50) { LOGGER()->error("TofinoMeshServer::FileFrameMockSource::preload_all_frames", "header/file mismatch %s\n", name); }
			}
			LOGGER()->error("TofinoMeshServer::FileFrameMockSource::preload_all_frames", "mock_source: file %s %d %d read\n", name, h_info[0], h_info[1]);

			m_frames.emplace_back(std::move(readbuf));
		}
		else
		{
			LOGGER()->error("TofinoMeshServer::FileFrameMockSource::preload_all_frames", "mock_source: file %s couldn't be opened (might be missing?)\n", name);
		}
	}
}

HRESULT TofinoMeshServer::FileFrameMockSource::GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber)
{
	auto delta_time_sec = prec_time::as_sec(m_start_time, prec_time::now());
	auto frames = static_cast<int>(delta_time_sec * 30.0);

	if (m_frames.size() == 0) { *ppData = 0; *pcbData = 0; *pdFrameNumber = 0; return E_FAIL; }
	m_frame_num = frames % m_frames.size();

	auto& cur_frame = m_frames[m_frame_num];
	if (cur_frame.size() == 0) { *ppData = 0; *pcbData = 0; *pdFrameNumber = m_frame_num; return S_OK; }

	auto intptr = reinterpret_cast<int*>(&cur_frame.front());
	*ppData = reinterpret_cast<const BYTE*>(&cur_frame.front());
	*pcbData = cur_frame.size();
	*pdFrameNumber = intptr[1];

	if (cur_frame.size() > sizeof(MeshHeaderVersion))
	{
		MeshHeaderVersion* tmh = reinterpret_cast<MeshHeaderVersion*>(&cur_frame.front());
		LOGGER()->error("TofinoMeshServer::FileFrameMockSource::preload_all_frames", "frame requested: current frame (%d) size:%d frame:%d color %dx%d\n", m_frame_num, intptr[0], intptr[1], tmh->color_data_width, tmh->color_data_height);
	}
	else
	{
		LOGGER()->error("TofinoMeshServer::FileFrameMockSource::preload_all_frames", "frame requested: current frame (%d) %d %d\n", m_frame_num, intptr[0], intptr[1]);
	}


	return S_OK;
}

