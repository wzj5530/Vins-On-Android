//
// Created by win7 on 2017/12/18.
//

#ifndef ARDEMO_LOG_UTIL_H
#define ARDEMO_LOG_UTIL_H

#define DEBUG 1


#ifdef _WINDOWS_		// allow this file to be included in PC projects
#define LOG( ... ) printf( __VA_ARGS__ );printf("\n")
#else

#include <android/log.h>
#include <stdlib.h>

#include <stdio.h>

static  bool logflag ;

void InitLogUtils();
void LogWithTag(int prio, const char * tag, const char * fmt, ...);
void LogWithTagWarn(int prio, const char * TAG, const char * tag, const char * fmt, ...);

void LogWithFileTag(int prio, const char * fileTag, const char * fmt, ...);
void LogWithFileTagWarn(int prio, const char * fileTag, const char * TAG, const char * fmt, ...);

#define VINS_LOG(...) LOGIF(__VA_ARGS__)
#define LOGIF( ... ) LogWithFileTag( ANDROID_LOG_INFO, __FILE__,__VA_ARGS__ )
#define LOG( ... ) LogWithFileTag( ANDROID_LOG_INFO, __FILE__,__VA_ARGS__ )
#define LOGD( ... ) LogWithFileTag( ANDROID_LOG_INFO, __FILE__,__VA_ARGS__ )
#define LOGWF(__TAG__, ... ) LogWithFileTagWarn(ANDROID_LOG_WARN,__FILE__,__TAG__, __VA_ARGS__ )
#define LOGEF( ... ) LogWithFileTag(ANDROID_LOG_ERROR,__FILE__,__VA_ARGS__ )

#define FAIL( ... ) {LOGEF(__VA_ARGS__ );abort();}

#define LOGI(__tag__, ... ) LogWithTag( ANDROID_LOG_INFO, __tag__, __VA_ARGS__ )
#define LOGW(__TAG__,__tag__, ... ) LogWithTagWarn(ANDROID_LOG_WARN,__TAG__,__tag__, __VA_ARGS__ )
#define LOGE(__tag__, ... ) LogWithTag(ANDROID_LOG_ERROR, __tag__, __VA_ARGS__ )


#if defined( ALLOW_LOG_SPAM )
#define SPAM(...) LogWithTag( ANDROID_LOG_VERBOSE, "Spam", __VA_ARGS__ )
#else
#define SPAM(...) { }
#endif

#endif	// ! _WINDOWS_

#endif //ARDEMO_LOG_UTIL_H
