#ifndef LOG_H
#define LOG_H

#include "spdlog/spdlog.h"

namespace ORB_SLAM3
{
class Log
{
private:
    static std::shared_ptr<spdlog::logger> mLog;

public:
    static std::shared_ptr<spdlog::logger> GetLog();
};
}

#endif
