//
// Created by win7 on 2017/12/22.
//

#ifndef ARDEMO_VINS_TYPE_H
#define ARDEMO_VINS_TYPE_H

//#include "feature_tracker.hpp"
//#include "global_param.hpp"
//#include "VINS.hpp"
#include "eigen3/Eigen/Eigen"
#include <map>

using namespace Eigen;
using namespace std;
struct IMU_MSG {
    double header;
    Vector3d acc;
    Vector3d gyr;
};

struct IMG_MSG {
    double header;
    map<int, Vector3d> point_clouds;
};




struct VINS_DATA_CACHE {
    double header;
    Vector3f P;
    Matrix3f R;
};

#endif //ARDEMO_VINS_TYPE_H
