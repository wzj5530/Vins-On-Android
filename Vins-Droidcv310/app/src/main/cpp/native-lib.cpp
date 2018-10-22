#include <jni.h>
#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>
#include <android/log.h>
#include "../a2ir/phone_sensor.h"
#include "../vins_estimator/src/estimator.h"
#include <time.h>
#include <timer.h>
#include "../a2ir/log_util.h"


using namespace cv;
using namespace std;

static a2ir::PhoneSensor *phoneSensorInstance = NULL;
extern Estimator estimator;
static bool bVinsInitilized = false;

extern "C" {
jstring Java_com_martin_ads_testopencv_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

void
Java_com_martin_ads_testopencv_MainActivity_nativeProcessFrame(JNIEnv *, jobject, jlong ptrGray,
                                                               jlong ptrRgba) {
    double header = PVR::Timer::GetSeconds();
    Mat &mGray = *(Mat *) ptrGray;
    Mat &mRgb = *(Mat *) ptrRgba;
    if (bVinsInitilized) {
        receiveImg(header, mGray, mRgb);
    }
}

JNIEXPORT jboolean
JNICALL
Java_com_martin_ads_testopencv_MainActivity_nativeVinsInitialise(
        JNIEnv *env,
        jobject obj) {

    vins_init("/sdcard/VINS/euroc_config_pixel.yaml");
    //vins_init("/sdcard/VINS/euroc_config.yaml");
    phoneSensorInstance = a2ir::PhoneSensor::GetInstance();
    bVinsInitilized = true;
    return true;
}
JNIEXPORT jboolean
JNICALL
Java_com_martin_ads_testopencv_MainActivity_nativeVinsStartSensor(
        JNIEnv *env,
        jobject obj) {
    if (NULL != phoneSensorInstance)
        phoneSensorInstance->StartTrack();
    LOGD("nativeVinsStartSensor");
    return true;
}
JNIEXPORT jboolean
JNICALL
Java_com_martin_ads_testopencv_MainActivity_nativeVinsStopSensor(
        JNIEnv *env,
        jobject obj) {

    //TBD:: much more works
    if (NULL != phoneSensorInstance)
        phoneSensorInstance->StopTrack();

    return true;
}
JNIEXPORT jboolean
JNICALL
Java_com_martin_ads_testopencv_MainActivity_nativeVinsQuit(
        JNIEnv *env,
        jobject obj) {


    delete phoneSensorInstance;
    phoneSensorInstance = NULL;

    LOGD("nativeVinsQuit");
    return true;
}



JNIEXPORT void
JNICALL
Java_com_martin_ads_testopencv_MainActivity_panGestureRecognizer(
        JNIEnv *env,
        jobject obj, jdouble vx_smooth, jdouble vy_smooth) {

    LOGD("zzzz Java_com_martin_ads_testopencv_MainActivity_panGestureRecognizer");

    estimator.drawresult.theta += vy_smooth / 100.0;
    estimator.drawresult.theta = fmod(estimator.drawresult.theta, 360.0);
    estimator.drawresult.phy += vx_smooth / 100.0;
    estimator.drawresult.phy = fmod(estimator.drawresult.phy, 360.0);
    estimator.drawresult.change_view_manualy = true;


}
JNIEXPORT void
JNICALL
Java_com_martin_ads_testopencv_MainActivity_scaleGestureRecognizer(
        JNIEnv *env,
        jobject obj, jdouble _scale) {

    LOGD("zzzz Java_com_martin_ads_testopencv_MainActivity_scaleGestureRecognizer %f", _scale);

    estimator.drawresult.change_view_manualy = true;


    if(estimator.drawresult.radius > 5 || _scale < 0)
        estimator.drawresult.radius -= _scale * 0.5;
    else {
        estimator.drawresult.Fx *= _scale;
        if (estimator.drawresult.Fx < 50)
            estimator.drawresult.Fx = 50;
        estimator.drawresult.Fy *= _scale;
        if (estimator.drawresult.Fy < 50)
            estimator.drawresult.Fy = 50;
    }
}

}//extern C



