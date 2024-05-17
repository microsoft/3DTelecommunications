#pragma once

#include <chrono>
#include <tuple>
#include <map>
#include <string>
#include "Logger.h"

namespace prec_time
{
	typedef std::chrono::high_resolution_clock							clock;
	typedef std::chrono::time_point<std::chrono::high_resolution_clock> timepoint;
	typedef std::pair<timepoint, timepoint> timerange;
	typedef LARGE_INTEGER tic_t;
	timepoint now();

	double as_sec(timepoint t0, timepoint t1);
	double as_ms(timepoint t0, timepoint t1);
	double as_us(timepoint t0, timepoint t1);

	tic_t tic();
	// Synonym for tic()
	tic_t toc();

	void sleep(int msec);
}

struct TimingData {
	std::string message;
	prec_time::tic_t start;
	prec_time::tic_t end;
	unsigned long order;

	double Duration();
};

class TimingTable {
public:
	TimingTable(const std::string& prefix = "", bool auto_log_upon_delete = false, Logger::Verbosity auto_log_level = Logger::Warning);
	~TimingTable();

	void Start(const std::string& key, const std::string& message = "");
	void End(const std::string& key);
	void SetMessage(const std::string& key, const std::string& message);

	std::map<std::string, TimingData>::iterator& IterBegin();
	std::map<std::string, TimingData>::iterator& IterEnd();

	void LogTimes(Logger::Verbosity level);
	void SetPrefix(const std::string& prefix) {
		m_prefix = prefix;
	}

private:
	std::map<std::string, TimingData> time_table;
	std::string m_prefix;
	bool m_auto_log;
	Logger::Verbosity m_auto_log_level;
	unsigned long m_order;
};
