#include "logging.h"


std::unordered_map<std::string, int> Logger::LOG_LEVEL_MAP_ = {
    {"QUIET", QUIET},
    {"KEY", KEY},
    {"VERBOSE", VERBOSE},
};

void CustomPrefix(std::ostream &s, const google::LogMessageInfo &l, void*) {
    s << l.severity
    << " id:"
    << l.thread_id << std::setfill('0')
    << " "
    << l.filename << ':' << l.line_number << "]";
 }

void Logger::setLogger(const char* const *argv, const std::string &log_verbosity){

    std::cout << "Logging system is being initialized" << std::endl;
    google::InitGoogleLogging(argv[0], &CustomPrefix);
    FLAGS_logtostderr = 1;
    FLAGS_v = LOG_LEVEL_MAP_[log_verbosity];

    LOG(INFO) << "the level of log verbosity is set to " << log_verbosity;
    LOG(INFO) << GREEN << "glog has been initialized" << RESET;

}


void Logger::setLogVerbosity(const std::string &log_verbosity){

    FLAGS_v = LOG_LEVEL_MAP_[log_verbosity];
    LOG(INFO) << "the level of log verbosity is set to " << log_verbosity;

}



