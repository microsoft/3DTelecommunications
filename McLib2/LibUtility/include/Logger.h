#ifndef	 __LOGGER_H__
#define __LOGGER_H__
#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>

#include <fstream>
#include <iostream>
#include <cstdarg>
#include <string>
#include <mutex>

#ifdef _WIN32

#define FW_ FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE
#define FR_ FOREGROUND_RED
#define FG_ FOREGROUND_GREEN
#define FB_ FOREGROUND_BLUE
#define FY_ FOREGROUND_RED | FOREGROUND_GREEN
#define FC_ FOREGROUND_GREEN | FOREGROUND_BLUE
#define FM_ FOREGROUND_BLUE | FOREGROUND_RED 
#define FWI_ FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY
#define FRI_ FOREGROUND_RED | FOREGROUND_INTENSITY
#define FGI_ FOREGROUND_GREEN | FOREGROUND_INTENSITY
#define FBI_ FOREGROUND_BLUE | FOREGROUND_INTENSITY
#define FYI_ FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY
#define FCI_ FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY
#define FMI_ FOREGROUND_BLUE | FOREGROUND_RED | FOREGROUND_INTENSITY
#define FNULL_ 0
#define BW_ BACKGROUND_RED | BACKGROUND_GREEN | BACKGROUND_BLUE
#define BR_ BACKGROUND_RED
#define BG_ BACKGROUND_GREEN
#define BB_ BACKGROUND_BLUE
#define BY_ BACKGROUND_RED | BACKGROUND_GREEN
#define BC_ BACKGROUND_GREEN | BACKGROUND_BLUE
#define BM_ BACKGROUND_BLUE | BACKGROUND_RED 
#define BWI_ BACKGROUND_RED | BACKGROUND_GREEN | BACKGROUND_BLUE | BACKGROUND_INTENSITY
#define BRI_ BACKGROUND_RED | BACKGROUND_INTENSITY
#define BGI_ BACKGROUND_GREEN | BACKGROUND_INTENSITY
#define BBI_ BACKGROUND_BLUE | BACKGROUND_INTENSITY
#define BYI_ BACKGROUND_RED | BACKGROUND_GREEN | BACKGROUND_INTENSITY
#define BCI_ BACKGROUND_GREEN | BACKGROUND_BLUE | BACKGROUND_INTENSITY
#define BMI_ BACKGROUND_BLUE | BACKGROUND_RED | BACKGROUND_INTENSITY 
#define BNULL_ 0

#endif

#define FPRINTF_MAX_BUF 1024

#define LOGGER() Logger::get_logger()

/**
*   Singleton Logger Class. Adapted  from: https://cppcodetips.wordpress.com/2014/01/02/a-simple-logger-class-in-c/
*
*   IMPORTANT: Always call LOGGER()->close() at the end of the program.
*/
class Logger
{
public:
    // The verbosity levels are sorted by verbosity level. If Debug is chosen, then 
    // every message from a lower level of verbosity is going to be printed, for instance.
    enum Verbosity {
        Fatal = 0,
        Error,
        Warning,
        Info,
        Debug,
        Trace
    };

    /*
    * Class used to signal the end of line for logging. Logger::endl *must* be used in conjunction with the other << operators, always at the end of the line.
    * Example:
    *
    * *LOGGER() << Logger::Fatal << "Testing " << 123 << Logger::endl;
    *
    * IMPORTANT: when using operators << , the first value that MUST be used is the verbosity level and the last MUST be Logger::endl.
    * Note that the LOGGER() pointer must be dereferenced as *LOGGER().
    * 
    * VERY IMPORTANT: one MUST user Logger::endl at the end of streams, otherwise LOGGER WILL
    * REMAIN IN A LOCKED STATE!
    */
    class LoggerEndl {
    public:
    };

    static LoggerEndl endl;

    static const std::string current_data_time();

    void set_filename(const std::string& filename);
    void set_error_filename(const std::string& filename);

    /*
     * IMPORTANT: always call close() a the exit of the software to close files that were possibly opened by the logger.
     */
    void close();

    /**
    *   Funtion to create the instance of logger class.
    *   @return singleton object of Clogger class..
    */
    static Logger* get_logger();

    static Verbosity get_verbosity();
    static void set_verbosity(Verbosity level);
    static bool check_verbosity(Verbosity level);

    /**
    *   Logs a message
    *   @param sMessage message to be logged.
    */
    void log(Logger::Verbosity level, const std::string& sMessage);

    /**
    *   Variable Length Logger function, without and with colored output
    *   @param format string for the message to be logged.
    */
    void log(Logger::Verbosity level, const char* format, ...);
#ifdef _WIN32
    void log(Logger::Verbosity level, WORD attrib, const char* format, ...);
#endif

    void fatal(const char* function_name, const char* format, ...);
    void error(const char* function_name, const char* format, ...);
    void warning(const char* function_name, const char* format, ...);
    void info(const char* format, ...);
    void debug(const char* format, ...);
    void trace(const char* format, ...);

    void high_priority(const char* format, ...);

    /**
    *   << overloaded function to log a message
    *   @param sMessage message to be logged.
    */
    Logger& operator<<(Verbosity level);
    Logger& operator<<(const std::string& value);
    Logger& operator<<(const char* value);
    Logger& operator<<(const char value);
    Logger& operator<<(const int value);
    Logger& operator<<(const unsigned int value);
    Logger& operator<<(const float value);
    Logger& operator<<(const double value);
    Logger& operator<<(const long value);
    Logger& operator<<(const unsigned long value);
    Logger& operator<<(const LoggerEndl& endl);

private:
    /**
    *    Default constructor for the Logger class.
    */
    Logger();
    /**
    *   copy constructor for the Logger class.
    */
    Logger(const Logger&) {};             // copy constructor is private
    /**
    *   assignment operator for the Logger class.
    */
    Logger& operator=(const Logger&) { return *this; };  // assignment operator is private

    static Verbosity m_Verbosity;
    /**
    *   Log file name.
    **/
    static std::string m_sFilename;
    static std::string m_sFilename_error;
    static FILE* m_Logfile_f;
    static FILE* m_Logfile_f_error;
    /**
    *   Singleton logger class object pointer.
    **/
    static Logger* m_pThis;
    /**
    *   Log file stream object.
    **/
    static Verbosity m_Current_stream_msg_verbosity;
    /**
    * Since Logger may be called from different threads simultaneously, we must ensure
    * that stream operations (<<) perform atomically to avoid the threads from selecting
    * different verbosity levels for the current stream. 
    * VERY IMPORTANT: one MUST user Logger::endl at the end of streams, otherwise LOGGER WILL 
    * REMAIN IN A LOCKED STATE!
    */
    static std::mutex  m_Current_stream_msg_mutex;

    void close_out();
    void close_error();

    void log_aux(Logger::Verbosity level, const char* function_name, const char* format, va_list args);
#ifdef _WIN32
    void log_aux(Logger::Verbosity level, WORD attrib, const char* function_name, const char* format, va_list args);
#endif

    void stream_operator_aux(const char* msg);
};


#endif //__LOGGER_H__