//////////////////////////////////
// Created by win7 on 2017/12/18.
// created by kevin.wang
///////////////////////////////////

#ifndef ARDEMO_PHONE_SENSOR_H
#define ARDEMO_PHONE_SENSOR_H

#include <errno.h>
#include <android/sensor.h>
#include <stdio.h>         //for file operation
#include <pthread.h>
#include "base_sensor.h"
#include "log_util.h"
//#include "vins/vins_type.h"
#include "vins_type.h"

namespace a2ir {
    class PhoneSensor: public BaseSensor{
    public:
        static PhoneSensor* GetInstance() {
            if (!mpInstance)
                mpInstance = new PhoneSensor();
            return mpInstance;
        }
        //PhoneSensor();
        virtual ~PhoneSensor();

        //BaseSensor
        void StartTrack();
        void ResetTrack();
        void StopTrack();

        bool IsGyroscopeAvailable();

        //Sensor Thread
        static void* SensorThreadMain(void* arg);
        static int SensorCallback(int fd, int events, void* data);
        static void HandleAccelerometerEvent(ASensorEvent& sensorEvent);
        static void HandleGyroscopeEvent(ASensorEvent& sensorEvent);




    private:
        PhoneSensor();
        PhoneSensor(PhoneSensor const&);// copy ctor is hidden
        PhoneSensor& operator=(PhoneSensor const&);  //assign op is hidden
        static PhoneSensor* mpInstance;

        //ASensor
        ASensorRef mAccSensorRef;
        ASensorRef mGyroSensorRef;
        ASensorManager* mpSensorManager;
        ASensorEventQueue* mpSensorEventQueue;

        //Sensor Thread
        pthread_t       mSensorThread;
        bool            mSensorThreadExit;
        ALooper*        mpSensorThreadLooper;
        pthread_cond_t  mSensorThreadReadyCv;
        pthread_mutex_t mSensorThreadReadyMutex;
        bool            mSensorThreadReady;

        IMU_MSG         acc_gro;

        //Data Process

        float mTimeDeltaMs;
        int64_t mLastGyroTimestampNs;
        bool mSensorThreadRunning;


    };

} /* namespace PVR */
#endif //ARDEMO_PHONE_SENSOR_H
