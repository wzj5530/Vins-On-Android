//
//  draw_result.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/11/16.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef draw_result_hpp
#define draw_result_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <camodocal/camera_models/Camera.h>
#include "../utility/utility.h"
//#include "global_param.hpp"
#include "delaunay.h"

//#include <android/log.h>
//#define LOG_TAG "draw_rsesult"
//#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#include "../a2ir/log_util.h"

using namespace cv;
using namespace Eigen;
using namespace std;

#define HEIGHT 640//480
#define WIDTH 360//640

#define RIC_y ((double)0.0)
#define RIC_p ((double)0.0)
#define RIC_r ((double)180.0)

#define TIC_X ((double)0.0)
#define TIC_Y ((double)0.092)
#define TIC_Z ((double)0.01)

//TODO::need camera pragram (default S8)
//extern camodocal::CameraPtr m_camera;
//s8
//#define FOCUS_LENGTH_X ((double)446.07)
//#define FOCUS_LENGTH_Y ((double)444.5)
//#define PX ((double)305.95)
//#define PY ((double)221.39)

//pixel
#define FOCUS_LENGTH_X ((double)479.88694)
#define FOCUS_LENGTH_Y ((double)479.50323)
#define PX ((double)321.19235)
#define PY ((double)237.32735)


#define C_PI 3.1415926
#define WINDOW_SIZE 10

//case iPhone7P:
//        printf("Device iPhone7 plus param\n");
//FOCUS_LENGTH_X = 526.600;
//FOCUS_LENGTH_Y = 526.678;
//PX = 243.481;
//PY = 315.280;
//
//SOLVER_TIME = 0.06;
//FREQ = 3;
//
//TIC_X = 0.0;
//TIC_Y = 0.092;
//TIC_Z = 0.01;
//return true;
//break;

//extrinsic param
//extern double TIC_X;
//extern double TIC_Y;
//extern double TIC_Z;
//
//extern double FOCUS_LENGTH_Y;
//extern double PY;
//extern double FOCUS_LENGTH_X;
//extern double PX;


struct GroundPoint
{
    int idx;
    Vector3f center;
    bool boxflag;
    bool moveflag;
    Vector3f ori, cox, coy, coz;
    Vector3f lix, liy, liz;
    float size;
    Vector4f initPlane;
    
    GroundPoint(int idx_, Vector3f center_)
    {
        idx = idx_;
        center = center_;
        boxflag = false;
        moveflag = false;
    }
};

class DrawResult
{
public:
    DrawResult(float _pitch, float _roll, float _yaw, float _Tx, float _Ty, float _Tz);
    
    float roll, pitch, yaw; //in degree
    float Tx, Ty, Tz; //in meters, only in z axis
    float radius, radiusAR;
    float theta, phy;
    float thetaAR, phyAR;
    float locationX, locationY;
    float locationX_p, locationY_p;
    float locationXT2, locationYT2;
    float locationXT2_p, locationYT2_p;
    float locationXP, locationYP;
    float locationXP_p, locationYP_p;
    float locationTapX, locationTapY;
    bool tapFlag;
    float locationLongPressX;
    float locationLongPressY;
    bool longPressFlag;
    
    
    float theta_p, phy_p, radius_p;
    float X0_p, Y0_p;
    int finger_state;
    int finger_s;
    int finger_d;
    int finger_p;
    
    //map<int, pair <int, pair< Vector3f, vector<Vector3f> > > > Ground;
    //map<int, pair <int, Vector3f > > Ground;
    vector<GroundPoint> Grounds;
    
    int Ground_idx;
    
    
    
    float lengthCube;
    vector<Vector3f> pose;
    vector<int> segment_indexs;
    float Fx,Fy;
    bool change_view_manualy;
    bool planeInit;
    bool startInit;
    Vector3f initPoint;
    Vector4f initPlane;
    Vector3f origin;
    Vector3f ConerX, ConerY, ConerZ;
    Vector3f lineX,lineY,lineZ;
    Vector3f origin_w;
    float X0, Y0;
    float X0AR, Y0AR;
    
    
    bool look_down;
    //for optical flow EKF
    cv::Mat pre_image;
    cv::Mat cur_image;
    vector<Point2f> pre_pts;
    vector<Point2f> cur_pts;
    vector<Point2f> n_pts;
    Point2f flow;  //velocity of the pixel
    bool KF_init;
    
    vector<Scalar> trajectory_color;
    vector<int> indexs;
    bool change_color;
    //4 sizes
    vector<Vector2f> pre_status, cur_status;
    vector<Matrix2f> K;
    vector<Matrix2f> cur_cov, pre_cov;
    void computeAR(vector<Vector3f> &point_cloud, Vector3f &model);
    
    void drawAR(cv::Mat &result, vector<Vector3f> &point_cloud, Vector3f P_latest, Matrix3f R_latest);
    void drawGround(cv::Mat &result, vector<Vector3f> &point_cloud, Vector3f P_latest, Matrix3f R_latest);
    void drawBox(cv::Mat &result, Vector3f corner_0, Vector3f corner_x, Vector3f corner_y, Vector3f corner_z, float size, Vector3f P_latest, Matrix3f R_latest, bool inAR);
    void Reprojection(cv::Mat &result, vector<Vector3f> &point_cloud, const Matrix3f *R_window,const Vector3f *T_window, bool box_in_trajectory);
    vector<Vector3f> calculate_camera_pose(Vector3f camera_center, Matrix3f Rc, float length);
    cv::Point2f World2VirturCam(Eigen::Vector3f xyz, float &depth);
    void drawBoxVirturCam(cv::Mat &result);
    void rejectWithF();
    cv::Scalar newColor();
private:
    Vector3f findZfromXY(Vector3f point, Vector4f plane);
    Vector4f findPlane(vector<Vector3f> &point_cloud);
    Vector3f findGround(vector<Vector3f> &point_cloud, vector<Vector3f> &inlier_points);
};

#endif /* draw_result_hpp */
