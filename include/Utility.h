#ifndef UTILITY_H
#define UTILITY_H

#include "opencv2/core.hpp"
#include <jni.h>
namespace ORB_SLAM2
{
cv::Mat GrayToARGB(const cv::Mat &gray);
jintArray MatToJintArray(const cv::Mat &mat, JNIEnv *env);

class JByteArrayToMat
{
public:
    JByteArrayToMat(jbyteArray byteArray, JNIEnv *env);
    ~JByteArrayToMat();
    cv::Mat operator()(cv::Size size, int type);

private:
    jbyteArray mByteArray;
    JNIEnv *mEnv;
    jbyte *mData = nullptr;
};
}

#endif
