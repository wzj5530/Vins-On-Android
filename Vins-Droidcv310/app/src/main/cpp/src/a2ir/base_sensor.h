//////////////////////////////////
// Created by win7 on 2017/12/18.
// created by kevin.wang
///////////////////////////////////

#ifndef ARDEMO_BASE_SENSOR_H
#define ARDEMO_BASE_SENSOR_H




namespace a2ir {

    class BaseSensor {
    public:
        virtual void StartTrack() = 0;

        virtual void ResetTrack() = 0;

        virtual void StopTrack() = 0;
    };
}
#endif //ARDEMO_BASE_SENSOR_H
