#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
//#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "../parameters.h"
#include "../utility/tic_toc.h"

//#include <android/log.h>
//#define LOG_TAG "tclog vins_estimator"
//#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#include "../a2ir/log_util.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;
using namespace cv;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);
//template <typename T> void reduceVector(vector<T> &v, vector<uchar> status);

struct max_min_pts{
	Point2f min;
	Point2f max;
};

class FeatureTracker
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeatureTracker();

    void readImage(const cv::Mat &_img, vector<Point2f> &good_pts, vector<int> &track_len);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    vector<cv::Point2f> undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<int> ids;    
    vector<int> track_cnt;
    camodocal::CameraPtr m_camera;

    static int n_id;

	vector<max_min_pts> parallax_cnt;
};
