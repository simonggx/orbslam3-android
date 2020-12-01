#include "Utility.h"
#include "Log.h"

#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"

namespace ORB_SLAM3
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
            argbPtr[j] = (0XFFU << 24U) |(grayValue << 16U) |  (grayValue << 8U) | ((grayValue));
        }
    }
    return argb;
}

cv::Mat BGRTOARGB(const cv::Mat &bgr)
{
    cv::Mat argb(bgr.size(), CV_8UC4);

    for (int i = 0; i < bgr.rows; ++i)
    {
        auto bgrPtr = bgr.ptr<cv::Vec3b>(i);
        auto argbPtr = argb.ptr<uint>(i);
        for (int j = 0; j < bgr.cols; ++j)
        {
            argbPtr[j] = (0XFFU << 24U) | (bgrPtr[j][2] << 16U) | (bgrPtr[j][1] << 8U) | (bgrPtr[j][0]);
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

JByteArrayToMat::JByteArrayToMat(jbyteArray byteArray, JNIEnv *env, const cv::Size &size):
    mByteArray(byteArray),
    mEnv(env),
    mSize(size)
{
     mData = mEnv->GetByteArrayElements(mByteArray, NULL);
}

cv::Mat JByteArrayToMat::operator()(int type)
{
    cv::Mat mat(mSize,type, (void *)mData);
    return mat;
}

JByteArrayToMat::~JByteArrayToMat()
{
    mEnv->ReleaseByteArrayElements(mByteArray, mData, 0);
}
}
