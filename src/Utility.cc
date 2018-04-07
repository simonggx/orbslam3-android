#include "Utility.h"
#include "Log.h"

#include "opencv2/imgproc.hpp"

namespace ORB_SLAM2
{
cv::Mat GrayToARGB(const cv::Mat &gray)
{
    cv::Mat argb(gray.size(), CV_8UC4);
    for (int i = 0; i < gray.rows; ++i)
    {
        auto grayPtr = gray.ptr<uchar>(i);
        auto argbPtr = argb.ptr<uint>(i);
        for (int j = 0; j < gray.cols; ++j)
        {
            //endian mode may change under jni
            uchar grayValue = grayPtr[j];
            argbPtr[j] = (0XFF << 24) |((grayValue & 0xFF) << 16) |  ((grayValue & 0xFF) << 8) | ((grayValue & 0xFF));
        }
    }
    return argb;
}

jintArray MatToJintArray(const cv::Mat &mat, JNIEnv *env)
{
    if (!mat.isContinuous())
    {
        Log::GetLog()->error("MatToJintArray do not support uncontinuous mat!");
        return nullptr;
    }
    if (CV_8UC4 != mat.type())
    {
        Log::GetLog()->error("MatToJintArray only support ARGB!");
        return nullptr;
    }
    size_t dataSize = mat.rows * mat.cols;
    jintArray intArray = env->NewIntArray(dataSize);
    env->SetIntArrayRegion(intArray, 0, dataSize, (const int32_t *)mat.data);
    return intArray;
}

JByteArrayToMat::JByteArrayToMat(jbyteArray byteArray, JNIEnv *env):
    mByteArray(byteArray),
    mEnv(env)
{
     mData = mEnv->GetByteArrayElements(mByteArray, NULL);
}

cv::Mat JByteArrayToMat::operator()(cv::Size size, int type)
{
    cv::Mat mat(size,type, (void *)mData);
    return mat;
}

JByteArrayToMat::~JByteArrayToMat()
{
    mEnv->ReleaseByteArrayElements(mByteArray, mData, 0);
}
}
