#ifndef SLAMJNI_H
#define SLAMJNI_H
#include <jni.h>
#include "opencv2/core.hpp"

namespace ORB_SLAM2
{
//jboolean InitORBSLAM();
//jbyteArray TrackMonocular(jbyteArray img);

void SaveImg(JNIEnv *env, jclass javaClass, jbyteArray img, jstring path);

jintArray ConvertGrayToARGB(JNIEnv *env, jclass javaClass, jbyteArray img);

jboolean InitSlam(JNIEnv *env, jclass javaClass, jstring vocPath, jstring settingPath);

jboolean DestroySlam(JNIEnv *env, jclass javaClass);

jintArray GrabImg(JNIEnv *env, jclass javaClass, jbyteArray img, jdouble timeStamp);
}

#endif
