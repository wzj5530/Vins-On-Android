
#include <jni.h>
#include <string>
#include <../a2ir/timer.h>
#include "../a2ir/jni_util.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "../a2ir/phone_sensor.h"
#include "../a2ir/log_util.h"
//#include "vins/vins_main.hpp"
#include "../vins_estimator/src/estimator.h"
using namespace cv;

/////////////////////////////////////////////////////////////////////////////////////////
/////////////  ---------    some global variables   -----------//////////////////////////
/////////////  ----     need to move to header files later on   ----- ///////////////////
/////////////////////////////////////////////////////////////////////////////////////////

const std::string global_version_num = "a2ir version: 00.171209.01";

static a2ir::PhoneSensor *phoneSensorInstance = NULL;

unsigned char *framebuffer;

extern Estimator estimator;

#define START_UP_VINS 1

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irInitialiseAR(
        JNIEnv *env,
        jobject obj) {
    VINS_LOG("tclog Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irInitialiseAR 0");
    if(START_UP_VINS)
    {
//        vins_init("/sdcard/VINS/euroc_config.yaml");
//        vins_init("./src/config/euroc/euroc_config_no_extrinsic.yaml");
        vins_init("/sdcard/VINS/euroc_config_no_extrinsic.yaml");
        VINS_LOG("tclog Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irInitialiseAR 1");
    }

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works


    return true;
}
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irChangeToResourcesDir(
        JNIEnv *env,
        jobject obj, jstring resourcesDirectoryPath) {
    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irChangeToResourcesDir");
    VINS_LOG("resourcesDirectoryPath = %s",resourcesDirectoryPath);
    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

    return true;
}

extern "C"
JNIEXPORT jstring JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irGetVersion(
        JNIEnv *env,
        jobject obj) {
    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irGetVersion");
    // NO TBD this function is completed
    return env->NewStringUTF(global_version_num.c_str());
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irStartRunning(
        JNIEnv *env,
        jobject obj, jstring vconf, jstring cparaName, float nearPlane, float farPlane) {

    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irStartRunning");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works


    phoneSensorInstance = a2ir::PhoneSensor::GetInstance();
    phoneSensorInstance->StartTrack();

    return true;
}

/////////////////////////////////////////////////////////////////////////////
///////////////a2irStopRunning is called in surfaceDestroyed/////////////////
/////////////////////////////////////////////////////////////////////////////
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irStopRunning(
        JNIEnv *env,
        jobject obj) {

    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irStopRunning");

    //TBD:: much more works

    phoneSensorInstance->StopTrack();
    delete phoneSensorInstance;
    phoneSensorInstance = NULL;
    return true;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irAcceptVideoImage(
        JNIEnv *env,
        jobject obj, jbyteArray pinArray, jint width, jint height, jint cameraIndex, jboolean cameraIsFrontFacing) {

//    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irAcceptVideoImage：%d * %d", width, height);

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

    size_t frame_size = width * height + 2 * width/2 * height/2;
    static int flag = 0;
    static int save_flag = 0;
    if(flag == 0)
    {
        framebuffer = (unsigned char*)calloc(frame_size, sizeof(unsigned char));
        flag =1;
    }

    double header = PVR::Timer::GetSeconds();

//    LOGD("xxxxxx imageTime = %f", header);

    env->GetByteArrayRegion(pinArray, 0, frame_size, (jbyte *)framebuffer);

    Mat Yuv (height + height/2, width, CV_8UC1, framebuffer);
    Mat Rgb;
    //cvtColor(Yuv,Rgb,CV_YUV2RGB_NV21);
//    cvtColor(Yuv,Rgb,CV_YUV2BGRA_NV21);
//    cvtColor(Yuv,Rgb,CV_YUV420sp2RGBA);
    cvtColor(Yuv,Rgb,CV_YUV2GRAY_NV21);

//    if (Yuv.type() != CV_8UC1 && Yuv.type() != CV_32FC1){
//        LOGD("xxxx image type wrong");
//    } else{
//        LOGD("xxxx image.type() ==== CV_8UC1");
//    }
//
//    if (Rgb.type() != CV_8UC1 && Rgb.type() != CV_32FC1){
//        LOGD("xxxx image type wrong");
//    } else{
//        LOGD("xxxx image.type() ==== CV_8UC1");
//    }

//    char filename[60];
//    std::sprintf(filename, "sdcard/VINS/androidImages/%.0f.png", (header * 1e13));
//    imwrite(filename, Rgb);

//#pragma mark - 存图
//    if(save_flag % 60 == 0)
//    {
//        //for (int i = 0; i < 20; ++i) {
//            imwrite("sdcard/VINS/androidImages/xxxx.png", Rgb);
//        //}
//        save_flag++;
////        save_flag = 1;
//    }

    if (Rgb.empty()){
        VINS_LOG("xxxx Rgb.empty()");
        return true;
    }

//    VINS_LOG("xxxx image = %s", filename);

#pragma mark - 时间戳
//    time_t rawtime;
//    struct tm * timeinfo;
//    char buffer [128];
//    time (&rawtime);
//    timeinfo = localtime (&rawtime);
//    strftime (buffer,sizeof(buffer),"Now is %Y/%m/%d %H:%M:%S",timeinfo);
//    VINS_LOG("xxxx image time buffer = %s", buffer);



    if(START_UP_VINS) {
//        VINS_LOG("timelog : receiveimage");
//        receiveImage(header, Rgb);
    }

    return true;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irCapture(
        JNIEnv *env,
        jobject obj) {
    // VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irCapture");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

    return true;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irUpdateAR(
        JNIEnv *env,
        jobject obj) {
    // VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irUpdateAR");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

    return true;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irUpdateDebugTexture(
        JNIEnv *env,
        jobject obj,jbyteArray pinArray,  jboolean flypY) {
    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irUpdateDebugTexture");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works


    return true;
}

extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irGetProjectionMatrix(
        JNIEnv *env,
        jobject obj) {
    // VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irGetProjectionMatrix");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

    /*float proj[16] = {0.8391f, 0.0f, 0.0f, 0.0f,
                      0.0f, 0.8391f, 0.0f, 0.0f,
                      0.0f, 0.0f, 1.0001f, -0.1f,
                      0.0f, 0.0f, 1.0f, 0.0f};*/
    float proj[16] = {2.1213257f, 0.0f, 0.0f, 0.0f,
                      0.0f, 2.6601632f, 0.0f, 0.0f,
                      0.004541695f, -0.0077143908f, -1.002002, -1.0f,
                      0.0f, 0.0f, -20.02002, 0.0f};
    return glArrayToJava(env, proj, 16);
    //if failure then return NULL;
}

extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irQueryMarkerTransformation(
        JNIEnv *env,
        jobject obj, jint markerUID) {
    // VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irQueryMarkerTransformation");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works


    if(estimator.solver_flag){ //NON_LINEAR

        float w  = estimator.poseResult[6];
        float x  = estimator.poseResult[3];
        float y  = estimator.poseResult[4];
        float z  = estimator.poseResult[5];
        float px  = estimator.poseResult[0] * (1);
        float py  = estimator.poseResult[1] * (1);
        float pz  = estimator.poseResult[2] * (1);

        float ww = w * w;
        float xx = x * x;
        float yy = y * y;
        float zz = z * z;
        float pose[16] = {(ww + xx - yy - zz), (2 * ( x * y - w * z )),(2 * ( x * z + w * y )) , 0.0f,
                          (2 * ( x * y + w * z )), (ww - xx + yy - zz), (2 * ( y * z - w * x )), 0.0f,
                          (2 * ( x * z - w * y )), (2 * ( y * z + w * x )), (ww - xx - yy + zz), 0.0f,
                          px / (-100.f), py/10.f, -317.08755f, 1.0f};
//        float pose[16] = {(1-2*yy-2*zz),(2*x*y+2*w*z),(2*x*z-2*w*y),0.0f,
//                           (2*x*y-2*w*z),(1-2*xx-2*zz),(2*y*z+2*w*x),0.0f,
//                           (2*x*z+2*w*y),(2*y*z-2*w*x),(1-2*xx-2*yy),0.0f,
//                          px / (-100.f), py/10.f, -317.08755f, 1.0f};

//        LOGD("xxxx pxpypz222 = %f, %f, %f", px, py, pz);

        for (int i = 0; i < 4; ++i) {
            LOGD("xxxx pose = %f, %f, %f, %f", pose[i*4], pose[i*4+1],pose[i*4+2],pose[i*4+3]);
        }


        return glArrayToJava(env, pose, 16);

    } else {  //INITIAL
        float pose1[16] = {0.278074f, 0.95854306f, -0.06221812f, 0.0f,
                           -0.74401f, 0.17394234f, -0.6451299f, 0.0f,
                           -0.6075627f, 0.22568668f, 0.76153255f, 0.0f,
                           -4.158332f, 9.809809f, -317.08755f, 1.0f};

        return glArrayToJava(env, pose1, 16);
    }
}




extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irShutdownAR(
        JNIEnv *env,
        jobject obj) {
    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irShutdownAR");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

    return true;
}
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irIsRunning(
        JNIEnv *env,
        jobject obj) {
    // VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irIsRunning");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

    return true;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irSetBorderSize(
        JNIEnv *env,
        jobject obj,jfloat size) {
    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irSetBorderSize");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

}


extern "C"
JNIEXPORT jfloat JNICALL
Java_com_hezhaoxin_solomon_vins_1android_baseactivity_NativeInterface_a2irGetBorderSize(
        JNIEnv *env,
        jobject obj) {
    VINS_LOG("Java_com_a2ir_ar_ardemo_baseactivity_NativeInterface_a2irGetBorderSize");

    //TBD:: much more works
    //TBD:: much more works
    //TBD:: much more works

    return 1.0f;
}


extern "C" {
jstring
Java_com_hezhaoxin_solomon_vins_1android_NDKloader_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "opencv图像处理";
    return env->NewStringUTF(hello.c_str());
}
jstring
Java_com_hezhaoxin_solomon_vins_1android_NDKloader_validate(
        JNIEnv *env,
        jobject,
        jlong addrGray,
        jlong addrRgba){
    std::string hello2 = "Hello from validate";
    return env->NewStringUTF(hello2.c_str());
}
}
//图像处理
extern "C"
JNIEXPORT jintArray JNICALL
Java_com_hezhaoxin_solomon_vins_1android_NDKloader_getGrayImage(
        JNIEnv *env,
        jclass type,
        jintArray pixels_,
        jint w, jint h) {
    jintArray result;
    return result;
}