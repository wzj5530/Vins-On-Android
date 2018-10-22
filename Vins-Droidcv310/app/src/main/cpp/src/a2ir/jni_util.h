//
// Created by win7 on 2017/12/16.
//

#ifndef ARDEMO_JNI_UTIL_H
#define ARDEMO_JNI_UTIL_H
#include <jni.h>
#include <string>

////////////////////////////////////////////////
jfloatArray glArrayToJava(JNIEnv *env, float *arr, int len) {
    jfloatArray result = NULL;
    if ((result = env->NewFloatArray(len))) env->SetFloatArrayRegion(result, 0, len, arr);
    return result;
}

//////////////////////////////////////////////////

#endif //ARDEMO_JNI_UTIL_H
