#include "SlamJni.h"

#include "Log.h"
#include "Utility.h"
#include "System.h"

#include <string>

#include "opencv2/imgcodecs.hpp"
namespace ORB_SLAM3
{
static ORB_SLAM3::System *orbSystem = nullptr;
static int orbWidth = 0;
static int orbHeight = 0;
    //int    cnt_image;//从文件中读视频流
    //std::ifstream ifile;//从文件中读视频流
    //cv::Mat M1l,M2l,M1r,M2r;
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

jboolean InitSlam(JNIEnv *env, jclass javaClass, jstring vocPath, jstring settingPath, jint width, jint height, jstring baseDir)
{
    std::string path("/storage/emulated/0/camoutput");
    const char *vocPathStr = env->GetStringUTFChars(vocPath, 0);
    const char *settingPathStr = env->GetStringUTFChars(settingPath, 0);
    const char *baseDirStr = env->GetStringUTFChars(baseDir, 0);
    Log::GetLog()->info("init orb slam, voc path: {}, setting path: {}, width: {}, height: {}", vocPathStr, settingPathStr, width, height);
    orbSystem=new ORB_SLAM3::System(vocPathStr,settingPathStr,ORB_SLAM3::System::MONOCULAR);
    //orbSystem = new System(vocPathStr, settingPathStr, path, System::STEREO , false, false);
    orbWidth = width;
    orbHeight = height;
    env->ReleaseStringUTFChars(vocPath, vocPathStr);
    env->ReleaseStringUTFChars(settingPath, settingPathStr);

    std::string cmd = "rm -rf /storage/emulated/0/camoutput";
    system(cmd.c_str());
    ::mkdir("/storage/emulated/0/camoutput", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    ::mkdir("/storage/emulated/0/camoutput/image", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    Log::GetLog()->info("init00000000000000000000000000000000");
    //cnt_image=0;
    /*
    //ifile.open("/storage/emulated/0/mav0/V101.txt");
    //双目相关
    //cv::FileStorage fsSettings("/storage/emulated/0/orb/EuRoC.yaml", cv::FileStorage::READ);
    cv::FileStorage fsSettings("/storage/emulated/0/orb/stereo1.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;
    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
       rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }
    //cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    //cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Log::GetLog()->info("init1111111111111111111111111111");
     */
    return true;
}

jboolean DestroySlam(JNIEnv *env, jclass javaClass)
{
    Log::GetLog()->info("destroy slam");
    if (orbSystem)
    {
        orbSystem->Shutdown();
    }
    //ifile.close();
    return true;
}

    jboolean ChangetoLOC_MODE(JNIEnv *env, jclass javaClass)
    {
        orbSystem->ActivateLocalizationMode();
        return true;
    }

    jboolean ChangetoSLAM_MODE(JNIEnv *env, jclass javaClass)
    {
        orbSystem->DeactivateLocalizationMode();
        return true;
    }

    jboolean Reset(JNIEnv *env, jclass javaClass)
    {
        orbSystem->DeactivateLocalizationMode();

        orbSystem->Reset();
        return true;
    }

jintArray GrabImg(JNIEnv *env, jclass javaClass, jbyteArray img, jdouble timeStamp)
{

    //////////////////////////////////////////////////////
    //单目从相机获取视频流
    JByteArrayToMat byteArrayToMat(img, env, cv::Size(orbWidth, orbHeight));
    cv::Mat gray = byteArrayToMat(CV_8UC1);
    cv::Mat frameBGR = orbSystem->TrackMonocular(gray, timeStamp);
    cv::Mat frameARGB = BGRTOARGB(frameBGR);
    return MatToJintArray(frameARGB, env);
    //////////////////////////////////////////////////////

    /*
     * JByteArrayToMat byteArrayToMat(img, env, cv::Size(orbWidth*2, orbHeight));
    cv::Mat gray = byteArrayToMat(CV_8UC1);
    cv::Rect left_rect(0, 0, 640, 480);//创建一个Rect框，属于cv中的类，四个参数代表x,y,width,height
    cv::Mat image_left;
    image_left = cv::Mat(gray, left_rect).clone();
    cv::Mat frameBGR = orbSystem->TrackMonocular(image_left, timeStamp);
    cv::Mat frameARGB = BGRTOARGB(frameBGR);
    return MatToJintArray(frameARGB, env);
     */

    /*
    Log::GetLog()->info("GrabImg00000000000000000000000000000000");
    //////////////////////////////////////////////////////////////////////////////////////////////
    //从相机获取双目视频流
    JByteArrayToMat byteArrayToMat(img, env, cv::Size(orbWidth*2, orbHeight));
    cv::Mat gray = byteArrayToMat(CV_8UC1);
    cv::Rect left_rect(0, 0, 320, 240);//创建一个Rect框，属于cv中的类，四个参数代表x,y,width,height
    cv::Rect right_rect(320, 0, 320, 240);
    cv::Mat image_left, image_right;
    image_left = cv::Mat(gray, left_rect).clone();
    image_right = cv::Mat(gray, right_rect).clone();
    Log::GetLog()->info("GrabImg111111111111111111111111111");
    cv::Mat frameBGR = orbSystem->TrackStereo(image_left,image_right, timeStamp);
    cv::Mat frameARGB = BGRTOARGB(frameBGR);
    Log::GetLog()->info("GrabImg22222222222222222222222222");
    return MatToJintArray(frameARGB, env);
    ////////////////////////////////////////////////////////////////////////////////////////////////
*/
    /*
     //从文件中读双目视频流
    char s1[102];
    if(ifile>>s1)
    {
        char src_path0[1024];
        sprintf(src_path0, "/storage/emulated/0/mav0/cam0/data/%s.png", s1);
        char src_path1[1024];
        sprintf(src_path1, "/storage/emulated/0/mav0/cam1/data/%s.png", s1);

        //cnt_image++;
        cv::Mat srcImg0 = cv::imread(src_path0);//读取图片
        cv::Mat srcImg1 = cv::imread(src_path1);//读取图片
        if(!srcImg0.empty() && !srcImg1.empty())
        {

            cv::Mat dstImg0;
            cv::cvtColor(srcImg0,dstImg0,CV_BGR2GRAY);
            cv::Mat dstImg1;
            cv::cvtColor(srcImg1,dstImg1,CV_BGR2GRAY);
            cv::Mat imLeftRect, imRightRect;
            cv::remap(dstImg0,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
            cv::remap(dstImg1,imRightRect,M1r,M2r,cv::INTER_LINEAR);
            //cv::Mat frameBGR = orbSystem->TrackMonocular(dstImg, timeStamp);
            //cv::Mat frameBGR = orbSystem->TrackStereo(imLeftRect,imRightRect, timeStamp);
            cv::Mat frameBGR = orbSystem->TrackStereo(dstImg0,dstImg1, timeStamp);
            cv::Mat frameARGB = BGRTOARGB(frameBGR);
            return MatToJintArray(frameARGB, env);
        }
    }
*/
}
}

static JNINativeMethod utilityMethods[] =
{
    {
        "convertGrayToARGB", "([BII)[I", (void *)(ORB_SLAM3::ConvertGrayToARGB)
    },
    {
        "saveImg", "([BLjava/lang/String;II)V", (void *)(ORB_SLAM3::SaveImg)
    }
};

static JNINativeMethod orbslamMethods[] =
{
    {
//        "initSlam", "(Ljava/lang/String;Ljava/lang/String;IILjava/lang/String;)Z", (void *)(ORB_SLAM3::InitSlam)
        "initSlam", "(Ljava/lang/String;Ljava/lang/String;IILjava/lang/String;)Z", (void *)(ORB_SLAM3::InitSlam)
    },
    {
        "destroySlam", "()Z", (void *)(ORB_SLAM3::DestroySlam)
    },
    {
            "changetoLOC_MODE", "()Z", (void *)(ORB_SLAM3::ChangetoLOC_MODE)
    },
    {
            "changetoSLAM_MODE", "()Z", (void *)(ORB_SLAM3::ChangetoSLAM_MODE)
    },
    {
            "reset", "()Z", (void *)(ORB_SLAM3::Reset)
    },
    {
        "grabImg", "([BD)[I", (void *)(ORB_SLAM3::GrabImg)
    }
};

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved)
{
    JNIEnv *env = nullptr;
    jint result = -1;

    if (vm->GetEnv((void **)&env, JNI_VERSION_1_4) != JNI_OK)
    {
        ORB_SLAM3::Log::GetLog()->error("jni 1.4 is not supported!");
        return result;
    }

    //register utility method
    jclass utilityClass = env->FindClass("cn/edu/hust/orb_slam/Utility");
    if (nullptr == utilityClass)
    {
        ORB_SLAM3::Log::GetLog()->error("can not find class cn/edu/hust/orb_slam/Utility");
        return result;
    }
    if (env->RegisterNatives(utilityClass, utilityMethods, sizeof(utilityMethods) / sizeof(utilityMethods[0])) < 0)
    {
        ORB_SLAM3::Log::GetLog()->error("register utility method fail!");
        return result;
    }

    //register orb method
    jclass orbslamClass = env->FindClass("cn/edu/hust/orb_slam/ORBSlam");
    if (nullptr == orbslamClass)
    {
        ORB_SLAM3::Log::GetLog()->error("can not find class cn/edu/hust/orb_slam/ORBSlam");
        return result;
    }
    if (env->RegisterNatives(orbslamClass, orbslamMethods, sizeof(orbslamMethods) / sizeof(orbslamMethods[0])) < 0)
    {
        ORB_SLAM3::Log::GetLog()->error("register orbslam method fail!");
        return result;
    }

    ORB_SLAM3::Log::GetLog()->info("register method succeed!");

    result = JNI_VERSION_1_4;
    return result;
}
