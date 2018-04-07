#include "Log.h"

namespace ORB_SLAM2
{
std::shared_ptr<spdlog::logger> Log::mLog = nullptr;

std::shared_ptr<spdlog::logger> Log::GetLog()
{
    if (!mLog)
    {
        mLog = spdlog::android_logger("ORB_SLAM2", "ORB_SLAM2");
    }
    return mLog;
}
}
