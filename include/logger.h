#ifndef _logger_h
#define _logger_h
//including the necessary header files
#include "log4cpp/Appender.hh"
#include "log4cpp/Category.hh"

#include <string>

class Logger
{
    private:
    

    protected:

    public:
        bool debug_mode;
        void log_error(std::string);
        void log_debug(std::string);
        void log_warn(std::string);
        void log_info(std::string);
        void change_log_file(std::string);
        void logger_initialize();
        void logger_initialize(int val);
        Logger();
        ~Logger();
        Logger(int val);
};

#endif 