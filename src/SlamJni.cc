#include "SlamJni.h"

#include "Log.h"
#include "Utility.h"
#include "System.h"

#include <string>

#include "opencv2/imgcodecs.hpp"
namespace ORB_SLAM2
{
static ORB_SLAM2::System *orbSystem = nullptr;
static int orbWidth = 0;
static int orbHeight = 0;

void SaveImg(JNIEnv *env, jclass javaClass, jbyteArray img, jstring path, jint width, jint height)
{
    const char *path_str = env->GetStringUTFChars(path, 0);
    JByteArrayToMat byteArrayToMat(img, env, cv::Size(width, height));
    cv::Mat gray = byteArrayToMat(CV_8UC1);
    Log::GetLog()->info("save image {}", path_str);
    cv::imwrite(path_str, gray);
    env->ReleaseStringUTFChars(path, path_str);
}

jintArray ConvertGrayToARGB(JNIEnv *env, jclass javaClass, jbyteArray img, jint width, jint height)
{
    JByteArrayToMat byteArrayToMat(img, env, cv::Size(width, height));
    cv::Mat gray = byteArrayToMat(CV_8UC1);
    cv::Mat ARGB = GrayToARGB(gray);
    return MatToJintArray(ARGB, env);
}

jboolean InitSlam(JNIEnv *env, jclass javaClass, jstring vocPath, jstring settingPath, jint width, jint height)
{
    const char *vocPathStr = env->GetStringUTFChars(vocPath, 0);
    const char *settingPathStr = env->GetStringUTFChars(settingPath, 0);
    Log::GetLog()->info("init orb slam, voc path: {}, setting path: {}, width: {}, height: {}", vocPathStr, settingPathStr, width, height);
    orbSystem = new System(vocPathStr, settingPathStr, System::MONOCULAR, true);
    orbWidth = width;
    orbHeight = height;
    env->ReleaseStringUTFChars(vocPath, vocPathStr);
    env->ReleaseStringUTFChars(settingPath, settingPathStr);
    return true;
}

jboolean DestroySlam(JNIEnv *env, jclass javaClass)
{
    Log::GetLog()->info("destroy slam");
    if (orbSystem)
    {
        orbSystem->Shutdown();
    }
    return true;
}

jintArray GrabImg(JNIEnv *env, jclass javaClass, jbyteArray img, jdouble timeStamp)
{
    JByteArrayToMat byteArrayToMat(img, env, cv::Size(orbWidth, orbHeight));
    cv::Mat gray = byteArrayToMat(CV_8UC1);
    orbSystem->TrackMonocular(gray, timeStamp);
    cv::Mat frameBGR = orbSystem->DrawFrame();
    cv::Mat frameARGB = BGRTOARGB(frameBGR);
    return MatToJintArray(frameARGB, env);
}
}

static JNINativeMethod utilityMethods[] =
{
    {
        "convertGrayToARGB", "([BII)[I", (void *)(ORB_SLAM2::ConvertGrayToARGB)
    },
    {
        "saveImg", "([BLjava/lang/String;II)V", (void *)(ORB_SLAM2::SaveImg)
    }
};

static JNINativeMethod orbslamMethods[] =
{
    {
        "initSlam", "(Ljava/lang/String;Ljava/lang/String;II)Z", (void *)(ORB_SLAM2::InitSlam)
    },
    {
        "destroySlam", "()Z", (void *)(ORB_SLAM2::DestroySlam)
    },
    {
        "grabImg", "([BD)[I", (void *)(ORB_SLAM2::GrabImg)
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

    //register orb method
    jclass orbslamClass = env->FindClass("cn/edu/hust/orb_slam/ORBSlam");
    if (nullptr == orbslamClass)
    {
        ORB_SLAM2::Log::GetLog()->error("can not find class cn/edu/hust/orb_slam/ORBSlam");
        return result;
    }
    if (env->RegisterNatives(orbslamClass, orbslamMethods, sizeof(orbslamMethods) / sizeof(orbslamMethods[0])) < 0)
    {
        ORB_SLAM2::Log::GetLog()->error("register orbslam method fail!");
        return result;
    }

    ORB_SLAM2::Log::GetLog()->info("register method succeed!");

    result = JNI_VERSION_1_4;
    return result;
}
