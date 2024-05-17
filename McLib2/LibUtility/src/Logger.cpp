#include "Logger.h"
#include <string.h>
#include <stack>
#include <time.h>
#include <sstream>
#include <assert.h>

using namespace std;

string Logger::m_sFilename = "stdout";
string Logger::m_sFilename_error = "stderr";
FILE* Logger::m_Logfile_f = stdout;
FILE* Logger::m_Logfile_f_error = stderr;
Logger* Logger::m_pThis = NULL;
Logger::Verbosity Logger::m_Verbosity = Logger::Fatal;
Logger::Verbosity Logger::m_Current_stream_msg_verbosity = Logger::Fatal;
Logger::LoggerEndl Logger::endl = Logger::LoggerEndl();
std::mutex Logger::m_Current_stream_msg_mutex;


Logger::Logger()
{

}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string Logger::current_data_time()
{
	time_t     now = time(NULL);
	struct tm  tstruct;
	char       buf[128];
#ifdef _WIN32
	localtime_s(&tstruct, &now);
#else
	localtime_r(&now, &tstruct);
#endif
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
	return buf;
}

Logger* Logger::get_logger() {
	if (m_pThis == NULL) {
		m_pThis = new Logger();

		if (m_sFilename == "" || m_Logfile_f == NULL) {
			m_Logfile_f = stdout;
			m_sFilename = "stdout";
		}

		if (m_sFilename_error == "" || m_Logfile_f_error == NULL) {
			m_Logfile_f_error = stderr;
			m_sFilename_error = "stderr";
		}
	}
	return m_pThis;
}

void Logger::set_filename(const std::string& filename) {
	close_out();

	if (filename == "") {
		m_Logfile_f = stdout;
		m_sFilename = "stdout";
	} else {
		m_sFilename = filename;
		m_Logfile_f = fopen(filename.c_str(), "w+");
		if (m_Logfile_f == NULL) {
			m_Logfile_f = stdout;
			m_sFilename = "stdout";
			fprintf(stderr, "Warning <Logger::set_filename>! Unable to open file %s to log output. Using stdout.", filename.c_str());
		}
	}
}

void Logger::set_error_filename(const std::string& filename) {
	close_error();

	if (filename == "") {
		m_Logfile_f_error = stderr;
		m_sFilename_error = "stderr";
	}
	else {
		m_sFilename_error = filename;
		m_Logfile_f_error = fopen(filename.c_str(), "w+");
		if (m_Logfile_f_error == NULL) {
			m_Logfile_f_error = stderr;
			m_sFilename_error = "stderr";
			fprintf(stderr, "Warning <Logger::set_error_filename>! Unable to open file %s to log errors. Using stderr.", filename.c_str());
		}
	}
}

void Logger::close() {
	close_out();
	close_error();
}

void Logger::close_out() {
	if (m_Logfile_f != NULL && m_Logfile_f != stdout && m_Logfile_f != stderr) {
		fclose(m_Logfile_f);
	}
}

void Logger::close_error() {
	if (m_Logfile_f_error != NULL && m_Logfile_f_error != stdout && m_Logfile_f_error != stderr) {
		fclose(m_Logfile_f_error);
	}
}
Logger::Verbosity Logger::get_verbosity() {
	return m_Verbosity;
}

void Logger::set_verbosity(Logger::Verbosity level) {
	m_Verbosity = level;
}

bool Logger::check_verbosity(Verbosity level) {
	return m_Verbosity >= level;
}

void Logger::log(Logger::Verbosity level, const char* format, ...) {
	va_list args;
	va_start(args, format);
	log_aux(level, NULL, format, args);
	va_end(args);
}

#ifdef _WIN32
void Logger::log(Logger::Verbosity level, WORD attrib, const char* format, ...)
{
	va_list args;
	va_start(args, format);
	log_aux(level, attrib, NULL, format, args);
	va_end(args);
}
#endif

void Logger::fatal(const char* function_name, const char* format, ...) {
	va_list args;
	va_start(args, format);
	std::string msg_prefix = "Fatal<";
	msg_prefix += function_name;
	msg_prefix += ">: ";
	log_aux(Logger::Fatal, msg_prefix.c_str(), format, args);
	va_end(args);
}

void Logger::error(const char* function_name, const char* format, ...) {
	va_list args;
	va_start(args, format);
	std::string msg_prefix = "Error<";
	msg_prefix += function_name;
	msg_prefix += ">: ";
	log_aux(Logger::Error, msg_prefix.c_str(), format, args);
	va_end(args);
}

void Logger::warning(const char* function_name, const char* format, ...) {
	va_list args;
	va_start(args, format);
	std::string msg_prefix = "Warning<";
	msg_prefix += function_name;
	msg_prefix += ">: ";
	log_aux(Logger::Warning, msg_prefix.c_str(), format, args);
	va_end(args);
}

void Logger::info(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log_aux(Logger::Info, NULL, format, args);
	va_end(args);
}

void Logger::debug(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log_aux(Logger::Debug, NULL, format, args);
	va_end(args);
}

void Logger::trace(const char* format, ...) {
	va_list args;
	va_start(args, format);
	log_aux(Logger::Trace, NULL, format, args);
	va_end(args);
}

void Logger::high_priority(const char* format, ...) {
	va_list args;
	va_start(args, format);
	std::string msg_prefix = "<High Priority -- Not an Error>: ";
	log_aux(Logger::Error, msg_prefix.c_str(), format, args);
	va_end(args);
}

#ifdef _WIN32
void Logger::log_aux(Logger::Verbosity level, const char* msg_prefix, const char* format, va_list args) {

	WORD attrib = BNULL_;

	// Choosing a default color for warnings/errors
	if (level <= Logger::Warning)
		attrib = FRI_;

	log_aux(level, attrib, msg_prefix, format, args);
}
#endif

#ifdef _WIN32
void Logger::log_aux(Logger::Verbosity level, WORD attrib, const char* msg_prefix, const char* format, va_list args) {
#else
void Logger::log_aux(Logger::Verbosity level, const char* msg_prefix, const char* format, va_list args) {
#endif

	if (level <= Logger::m_Verbosity) {
		std::lock_guard<std::mutex> guard(m_Current_stream_msg_mutex);

		char* sMessage = NULL;
#ifdef _WIN32
		HANDLE handle = 0;
#endif
		FILE* file = m_Logfile_f;
		if (level > Logger::Error)
		{
#ifdef _WIN32
			handle = ::GetStdHandle(STD_OUTPUT_HANDLE);
			::SetConsoleTextAttribute(handle, attrib);
#endif
		}
		else
		{
			file = m_Logfile_f_error;
#ifdef _WIN32
			handle = ::GetStdHandle(STD_ERROR_HANDLE);
			::SetConsoleTextAttribute(handle, attrib);
#endif
		}

		int nLength;
#ifdef _WIN32
		
		//  Return the number of characters in the string referenced the list of arguments.
		// _vscprintf doesn't count terminating '\0' (that's why +1)
		nLength = _vscprintf(format, args) + 1;
		sMessage = new char[nLength];
		vsprintf_s(sMessage, nLength, format, args);
#else
		nLength = vasprintf(&sMessage, format, args);

		if (nLength < 0) {
			exit(-1);
		}
#endif

		if (msg_prefix != NULL) {
			fprintf(file, "%s: %s: %s\n", current_data_time().c_str(), msg_prefix, sMessage);
		}
		else {
			fprintf(file, "%s: %s\n", current_data_time().c_str(), sMessage);
		}
		fflush(file);
		
#ifdef _WIN32
		::SetConsoleTextAttribute(handle, FW_ | BNULL_);
		delete[] sMessage;
#else
		free(sMessage);
#endif

	}
}

void Logger::log(Logger::Verbosity level, const string& sMessage)
{
	if (level <= Logger::m_Verbosity) {
		stream_operator_aux(sMessage.c_str());
	}

}

Logger& Logger::operator<<(Logger::Verbosity level)
{
	// IMPORANT: locking Logger until the stream finishes printing
	m_Current_stream_msg_mutex.lock();

	m_Current_stream_msg_verbosity = level;

	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		std::stringstream ss;
		ss << current_data_time() << ": ";
		switch (m_Current_stream_msg_verbosity)
		{
		case Logger::Fatal:
			ss << "<Fatal>: ";
			break;
		case Logger::Error:
			ss << "<Error>: ";
			break;
		case Logger::Warning:
			ss << "<Warning>: ";
			break;
		case Logger::Debug:
			ss << "<Debug>: ";
			break;
		case Logger::Trace:
			ss << "<Trace>: ";
			break;
		case Logger::Info:
		default:
			break;
		}

		stream_operator_aux(ss.str().c_str());
	}


	return *this;
}
Logger& Logger::operator<<(const string& value)
{
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		stream_operator_aux(value.c_str());
	}


	return *this;
}

Logger& Logger::operator<<(const char* value)
{
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		stream_operator_aux(value);
	}


	return *this;
}

Logger& Logger::operator<<(const char value)
{
	std::stringstream ss;
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		ss << value;
		stream_operator_aux(ss.str().c_str());
	}


	return *this;
}

Logger& Logger::operator<<(const int value)
{
	std::stringstream ss;
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		ss << value;
		stream_operator_aux(ss.str().c_str());
	}


	return *this;
}

Logger& Logger::operator<<(const unsigned int value)
{
	std::stringstream ss;
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		ss << value;
		stream_operator_aux(ss.str().c_str());
	}


	return *this;
}

Logger& Logger::operator<<(const float value)
{
	std::stringstream ss;
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		ss << value;
		stream_operator_aux(ss.str().c_str());
	}


	return *this;
}

Logger& Logger::operator<<(const double value)
{
	std::stringstream ss;
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		ss << value;
		stream_operator_aux(ss.str().c_str());
	}

	return *this;
}

Logger& Logger::operator<<(const long value)
{
	std::stringstream ss;
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		ss << value;
		stream_operator_aux(ss.str().c_str());
	}

	return *this;
}

Logger& Logger::operator<<(const unsigned long value)
{
	std::stringstream ss;
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		ss << value;
		stream_operator_aux(ss.str().c_str());
	}

	return *this;
}

Logger& Logger::operator<<(const LoggerEndl& endl) {
	std::stringstream ss;
	if (m_Current_stream_msg_verbosity <= Logger::m_Verbosity) {
		ss << std::endl;
		stream_operator_aux(ss.str().c_str());
	}

	m_Current_stream_msg_verbosity = Logger::Fatal;

	// This lock should have been called 
	m_Current_stream_msg_mutex.unlock();

	return *this;
}

void Logger::stream_operator_aux(const char* msg) {
	if (msg != NULL) {
		FILE* file = m_Logfile_f;
		if (m_Current_stream_msg_verbosity <= Logger::Error)
		{
			file = m_Logfile_f_error;
		}
		fprintf(file, "%s", msg);
		fflush(file);
	}
}

/*********************************************************************************************
//////////////////////////// print COLORED strings in the console ////////////////////////////
*********************************************************************************************/

#ifdef _WIN32

int fprintf2(WORD attrib, FILE* file, const char* format, ...)
{
	HANDLE handle = 0;
	if (DWORD(file) == DWORD(stdout))
	{
		handle = ::GetStdHandle(STD_OUTPUT_HANDLE);
		::SetConsoleTextAttribute(handle, attrib);
	}
	else if (DWORD(file) == DWORD(stderr))
	{
		handle = ::GetStdHandle(STD_ERROR_HANDLE);
		::SetConsoleTextAttribute(handle, attrib);
	}

	static char buf[FPRINTF_MAX_BUF];
	va_list args;
	int printed;
	va_start(args, format);
	vsprintf_s(buf, format, args);
	va_end(args);
	printed = ::fprintf(file, buf);

	if (handle != 0)
	{
		::SetConsoleTextAttribute(handle, FW_ | BNULL_);
	}
	return printed;
}

int printf2(WORD attrib, const char* format, ...)
{
	HANDLE handle = 0;
	handle = ::GetStdHandle(STD_OUTPUT_HANDLE);
	::SetConsoleTextAttribute(handle, attrib);
	static char buf[FPRINTF_MAX_BUF];
	va_list args;
	int printed;
	va_start(args, format);
	vsprintf_s(buf, format, args);
	va_end(args);
	printed = ::printf(buf);
	::SetConsoleTextAttribute(handle, FW_ | BNULL_);
	return printed;
}

#endif