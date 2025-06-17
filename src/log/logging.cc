#include "logging.h"


namespace modules_vins{
    

std::unordered_map<std::string, int> Logger::LOG_LEVEL_MAP_ = {
    {"QUIET", QUIET},
    {"KEY", KEY},
    {"VERBOSE", VERBOSE},
};



void Logger::setLogger(const char* const *argv, const std::string &log_verbosity){

    std::cout << "Logging system is being initialized" << std::endl;
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_v = LOG_LEVEL_MAP_[log_verbosity];

    LOG(INFO) << "the level of log verbosity is set to " << log_verbosity;
    LOG(INFO) << GREEN << "glog has been initialized" << RESET;

}



}  // namespace modules_vins


