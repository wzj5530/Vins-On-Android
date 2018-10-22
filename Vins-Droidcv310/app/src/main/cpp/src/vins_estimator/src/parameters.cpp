#include "parameters.h"
#include <string>
#include <unistd.h>
#include <iostream>

#include <android/log.h>
//#define LOG_TAG "vins_parameter"
//#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
//#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#include "../a2ir/log_util.h"
#define FILENAMEPATH_MAX 80
using namespace std;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
int LOOP_CLOSURE = 0;
int MIN_LOOP_NUM;
std::string CAM_NAMES_ESTIMATOR;   //add
std::string PATTERN_FILE;
std::string VOC_FILE;
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
int IMAGE_ROW, IMAGE_COL;
std::string VINS_FOLDER_PATH;
int MAX_KEYFRAME_NUM;

//feature tracker section
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE_FEATURE_TRACKER;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

void readParameters(const string & config_file)
{
    cv::FileStorage fsSettings(config_file.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        VINS_LOG("[ERROR] LoadConfigFile: Wrong path to settings : %s !", config_file.c_str());
    }

    VINS_FOLDER_PATH = "/sdcard/VINS/";
    IMAGE_COL = fsSettings["image_width"];
    IMAGE_ROW = fsSettings["image_height"];

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];

    fsSettings["output_path"] >> VINS_RESULT_PATH;
    VINS_RESULT_PATH = VINS_FOLDER_PATH + VINS_RESULT_PATH;
    VINS_LOG("[INFO] LoadConfigFile: VINS_RESULT_PATH : %s !", VINS_RESULT_PATH.c_str());

    FILE *fp = fopen(VINS_RESULT_PATH.c_str(),"w+");
    if (fp) {
        VINS_LOG("[INFO] LoadConfigFile: VINS_RESULT_PATH open success !");
        char buffer[88] = "open file:\n";
        memset(buffer,sizeof(char),sizeof(buffer));
        fwrite(buffer, sizeof(char), strlen(buffer), fp);
        fclose(fp);
    } else {
        VINS_LOG("[ERROR] LoadConfigFile: VINS_RESULT_PATH open failed  %s!", strerror(errno));
    }


    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    VINS_LOG("[INFO] LoadConfigFile: ACC_N:%f , ACC_W:%f ,GYR_N:%f , GYR_W:%f , G.z():%f",ACC_N, ACC_W, GYR_N, GYR_W,  G.z() );
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        VINS_LOG("[INFO] LoadConfigFile: ESTIMATE_EXTRINSIC == 2 ");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
        EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;
        VINS_LOG("[INFO] LoadConfigFile: EX_CALIB_RESULT_PATH : %s !", EX_CALIB_RESULT_PATH.c_str());

    }
    else
    {

        if ( ESTIMATE_EXTRINSIC == 1)
        {
            VINS_LOG("[INFO] LoadConfigFile: ESTIMATE_EXTRINSIC == 1 ");
            fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
            EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;
            VINS_LOG("[INFO] LoadConfigFile: EX_CALIB_RESULT_PATH : %s !", EX_CALIB_RESULT_PATH.c_str());
        }
        if (ESTIMATE_EXTRINSIC == 0)
            VINS_LOG("[INFO] LoadConfigFile: ESTIMATE_EXTRINSIC == 0 ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);


        VINS_LOG("[INFO] LoadConfigFile: Extrinsic_R : %f            %f               %f",  RIC[0](0,0), RIC[0](0,1), RIC[0](0,2));
        VINS_LOG("[INFO] LoadConfigFile: Extrinsic_R : %f            %f               %f",  RIC[0](1,0), RIC[0](1,1), RIC[0](1,2));
        VINS_LOG("[INFO] LoadConfigFile: Extrinsic_R : %f            %f               %f",  RIC[0](2,0), RIC[0](2,1), RIC[0](2,2));
        VINS_LOG("[INFO] LoadConfigFile: Extrinsic_T : %f   %f   %f", TIC[0].transpose()(0), TIC[0].transpose()(1), TIC[0].transpose()(2) );

    }

    LOOP_CLOSURE = fsSettings["loop_closure"];
    if (LOOP_CLOSURE == 1)
    {
        fsSettings["voc_file"] >> VOC_FILE;;
        fsSettings["pattern_file"] >> PATTERN_FILE;
        MIN_LOOP_NUM = fsSettings["min_loop_num"];
        CAM_NAMES_ESTIMATOR = config_file;   //add
    }


    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;
    MAX_KEYFRAME_NUM = 1000;

    // feature tracker
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "/src/config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);
    WINDOW_SIZE_FEATURE_TRACKER = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 479.46870;
    PUB_THIS_FRAME = false;

    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
    if (FREQ == 0)
        FREQ = 10;

    VINS_LOG("[INFO] LoadConfigFile: VOC_FILE: %s", VOC_FILE.c_str());
    VINS_LOG("[INFO] LoadConfigFile: PATTERN_FILE: %s", PATTERN_FILE.c_str());
    fsSettings.release();
}
