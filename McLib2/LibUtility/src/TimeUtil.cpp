#include "TimeUtil.h"
#include <vector>
#include <algorithm>

#ifdef WIN32
#include <Windows.h>
#endif

using namespace std::chrono;

namespace prec_time
{
#ifdef WIN32
	unsigned __int64 GetPerfCount() {
		unsigned __int64 val;
		QueryPerformanceCounter((LARGE_INTEGER*)&val);
		return val;
	}

	unsigned __int64 GetPerfFreq() {
		unsigned __int64 val;
		QueryPerformanceFrequency((LARGE_INTEGER*)&val);
		return val;
	}

	void TestTimes() {
		auto st1 = prec_time::clock::now();
		auto hxr1 = prec_time::now();
		Sleep(10000);
		auto st2 = prec_time::clock::now();
		auto hxr2 = prec_time::now();

		auto delta = (st2 - st1).count();
		auto deltaXnow = (hxr2 - hxr1).count();
	}
#endif

	timepoint now()
	{
#ifdef WIN32
		static timepoint		system_epoch = clock::now();
		static unsigned __int64 pcEpoch = GetPerfCount();
		static unsigned __int64 qpf = GetPerfFreq();
		static double dpcToSysTicks = (10000000.0 / qpf);

		unsigned __int64 val;
		QueryPerformanceCounter((LARGE_INTEGER*)&val);
		return system_epoch + clock::duration((clock::rep)((val - pcEpoch) * dpcToSysTicks));
#else
		return clock::now();
#endif
	}

	tic_t tic() {
		LARGE_INTEGER t;
		::QueryPerformanceCounter(&t);

		return t;
	}

	// Synonym for tic()
	tic_t toc() {
		return tic();
	}

	void sleep(int msec) {
#ifdef WIN32
#else
#endif
	}

	double as_sec(timepoint t0, timepoint t1) {
		return duration_cast<nanoseconds>(t1 - t0).count() * 1e-9;
	}

	double as_ms(timepoint t0, timepoint t1) {
		return duration_cast<nanoseconds>(t1 - t0).count() * 1e-6;
	}

	double as_us(timepoint t0, timepoint t1) {
		return duration_cast<nanoseconds>(t1 - t0).count() * 1e-3;
	}

	double as_ms(tic_t start, tic_t end) {
		LARGE_INTEGER freq;
		::QueryPerformanceFrequency(&freq);

		return (end.QuadPart - start.QuadPart) * 1000.0 / (double)freq.QuadPart;
	}

}


double TimingData::Duration() {
	return prec_time::as_ms(start, end);
}

TimingTable::TimingTable(const std::string& prefix, bool auto_log_upon_delete, Logger::Verbosity auto_log_level) : m_prefix(prefix),
	m_auto_log(auto_log_upon_delete), m_auto_log_level(auto_log_level), m_order(0) {

}

TimingTable::~TimingTable() {
	if (m_auto_log) {
		LogTimes(m_auto_log_level);
	}
}

void TimingTable::Start(const std::string& key, const std::string& message) {
	time_table[key].start = prec_time::tic();
	time_table[key].end.QuadPart = 0; // Signaling that the end time was not set
	time_table[key].message = message;
}

void TimingTable::End(const std::string& key) {
	time_table[key].end = prec_time::toc();
	time_table[key].order = m_order;
	m_order++;
}

void TimingTable::SetMessage(const std::string& key, const std::string& message) {
	time_table[key].message = message;
}

std::map<std::string, TimingData>::iterator& TimingTable::IterBegin() {
	return time_table.begin();
}

std::map<std::string, TimingData>::iterator& TimingTable::IterEnd() {
	return time_table.end();
}

void TimingTable::LogTimes(Logger::Verbosity level) {
	prec_time::tic_t default_end = prec_time::toc();
	std::vector<std::pair<std::string, TimingData> > sorted_times;

	for (auto it = IterBegin(); it != IterEnd(); it++) {
		// If, for some reason, the end timepoint was not set, we use now() -- prec_time::toc() -- 
		// as the default value and add append a warning to the message if m_auto_log is set to false.
		if (it->second.end.QuadPart <= 0) {
			it->second.end = default_end;
		}
		sorted_times.push_back(*it);
	}

	std::sort(sorted_times.begin(), sorted_times.end(), 
		[](std::pair<std::string, TimingData> a, std::pair<std::string, TimingData> b) 
		{
			return a.second.order < b.second.order; 
		});

	for (auto it = sorted_times.begin(); it != sorted_times.end(); it++) {
		std::string msg = (it->second.message != "") ? it->second.message : it->first;

		if (m_prefix != "")
			LOGGER()->log(level, "(%s) %s: %.6lfms. FPS: %.2lf", m_prefix.c_str(), msg.c_str(), it->second.Duration(), 1000.0f / it->second.Duration());
		else
			LOGGER()->log(level, "%s: %.6lfms. FPS: %.2lf", msg.c_str(), it->second.Duration(), 1000.0f / it->second.Duration());
	}

	// Since we have logged the times, there's no need to auto log anymore upon exit.
	// Note that this prevents us from doubling logging the times when the user invokes
	// TimingTable::LogTimes instead of deferring such call to the destructor 
	m_auto_log = false;
}