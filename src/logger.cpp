#include "logger.h"

// #include "log4cpp/config.h"
#include "log4cpp/Category.hh"
#include "log4cpp/Appender.hh"
#include "log4cpp/FileAppender.hh"
#include "log4cpp/OstreamAppender.hh"
#include "log4cpp/Layout.hh"
#include "log4cpp/BasicLayout.hh"
#include "log4cpp/Priority.hh"
#include <log4cpp/PropertyConfigurator.hh>

#include <stdio.h>
#include <string.h>
#include <fstream>

void Logger::log_error(std::string error_message)
{
	log4cpp::Category& sub1 = log4cpp::Category::getInstance(std::string("sub1"));
	sub1 << log4cpp::Priority::ERROR << error_message;
    
    return;
}
void Logger::log_debug(std::string debug_message)
{
	log4cpp::Category& sub1 = log4cpp::Category::getInstance(std::string("sub1"));
	sub1 << log4cpp::Priority::DEBUG << debug_message;
    
    return;
}
void Logger::log_warn(std::string warn_message)
{
	log4cpp::Category& sub1 = log4cpp::Category::getInstance(std::string("sub1"));
	sub1 << log4cpp::Priority::WARN << warn_message;
    
    return;
}
void Logger::log_info(std::string info_message)
{
	log4cpp::Category& sub1 = log4cpp::Category::getInstance(std::string("sub1"));
	sub1 << log4cpp::Priority::INFO << info_message;
    
    return;
}

void Logger::logger_initialize()
{
     std::string initFileName = "include/log4cpp.properties";
	try
    {
        log4cpp::PropertyConfigurator::configure(initFileName);
    }
    catch(log4cpp::ConfigureFailure &e )
    {
        std::cout<<"Error in opening log configuration file.\n This error will not be logged.\n";
        exit(0);
    }
    return;
}
Logger::Logger()
{
   logger_initialize();
}
