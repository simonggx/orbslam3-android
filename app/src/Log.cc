#include "Log.h"

namespace ORB_SLAM3
{
std::shared_ptr<spdlog::logger> Log::mLog = nullptr;

std::shared_ptr<spdlog::logger> Log::GetLog()
{
    if (!mLog)
    {
        mLog = spdlog::android_logger("ORB_SLAM3", "ORB_SLAM3");
    }
    return mLog;
}
}
