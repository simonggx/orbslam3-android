#include "SlamJni.h"

#include "Log.h"
#include "Utility.h"

#include <string>

#include "opencv2/imgcodecs.hpp"
namespace ORB_SLAM2
{

void SaveImg(JNIEnv *env, jclass javaClass, jbyteArray img, jstring path)
{
    const char *path_str = env->GetStringUTFChars(path, 0);
    JByteArrayToMat byteArrayToMat(img, env);
    cv::Mat gray = byteArrayToMat(cv::Size(640, 480), CV_8UC1);
    Log::GetLog()->info("save image {}", path_str);
    cv::imwrite(path_str, gray);
    env->ReleaseStringUTFChars(path, path_str);
}

jintArray ConvertGrayToARGB(JNIEnv *env, jclass javaClass, jbyteArray img)
{
    JByteArrayToMat byteArrayToMat(img, env);
    cv::Mat gray = byteArrayToMat(cv::Size(640, 480), CV_8UC1);
    cv::Mat ARGB = GrayToARGB(gray);
    return MatToJintArray(ARGB, env);
}
}

static JNINativeMethod utilityMethods[] =
{
    {
        "convertGrayToARGB", "([B)[I", (void *)(ORB_SLAM2::ConvertGrayToARGB)
    },
    {
        "saveImg", "([BLjava/lang/String;)V", (void *)(ORB_SLAM2::SaveImg)
    }
};

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved)
{
    JNIEnv *env = nullptr;
    jint result = -1;

    if (vm->GetEnv((void **)&env, JNI_VERSION_1_4) != JNI_OK)
    {
        ORB_SLAM2::Log::GetLog()->error("jni 1.4 is not supported!");
        return result;
    }

    //register utility method
    jclass utilityClass = env->FindClass("cn/edu/hust/orb_slam/Utility");
    if (nullptr == utilityClass)
    {
        ORB_SLAM2::Log::GetLog()->error("can not find class cn/edu/hust/orb_slam/Utility");
        return result;
    }
    if (env->RegisterNatives(utilityClass, utilityMethods, sizeof(utilityMethods) / sizeof(utilityMethods[0])) < 0)
    {
        ORB_SLAM2::Log::GetLog()->error("register utility method fail!");
        return result;
    }

    ORB_SLAM2::Log::GetLog()->info("register method succeed!");

    result = JNI_VERSION_1_4;
    return result;
}
