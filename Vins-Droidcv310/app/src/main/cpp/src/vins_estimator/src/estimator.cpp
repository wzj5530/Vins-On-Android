#include "estimator.h"
#include <boost/shared_array.hpp>
#include <android/log.h>
//#define LOG_TAG "estimator"
//#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
//#define LOGDTC(...) __android_log_print(ANDROID_LOG_DEBUG, "tclog", __VA_ARGS__)
#include "../a2ir/log_util.h"

Estimator::Estimator(): f_manager{Rs}
{
    //ROS_INFO("init begins");
    clearState();
    failure_occur = 0;
    poseResult[7] = {0};
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

//    t_drift.setZero();
//    r_drift.setIdentity();

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
//    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    relocalize = false;
    retrive_data_vector.clear();
    relocalize_t = Eigen::Vector3d(0, 0, 0);
    relocalize_r = Eigen::Matrix3d::Identity();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    VINS_LOG("[VINS] WorkerT: processIMU frame_count = %d", frame_count);
    VINS_LOG("[VINS] WorkerT: Bas[%d]=%f %f %f  Bgs[%d]=%f %f %f",
            frame_count, Bas[frame_count].x(), Bas[frame_count].y(), Bas[frame_count].z(),
            frame_count, Bgs[frame_count].x(), Bgs[frame_count].y(), Bgs[frame_count].z());
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::update_loop_correction() {
    //update loop correct pointcloud
    correct_point_cloud.clear();
    for (auto &it_per_id : f_manager.feature) {
//        LOGD("f_manager.feature.size() = %d",f_manager.feature.size());
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 4 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (/*it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 ||*/ it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d tmp = relocalize_r * Rs[imu_i] * (ric[0] * pts_i + tic[0]) + relocalize_r * Ps[imu_i] + relocalize_t;
        correct_point_cloud.push_back(tmp.cast<float>());
    }
    //update correct pose
    for (int i = 0; i < WINDOW_SIZE; i++) {
        Vector3d correct_p = relocalize_r * Ps[i] + relocalize_t;
        correct_Ps[i] = correct_p.cast<float>();
        Matrix3d correct_r = relocalize_r * Rs[i];
        correct_Rs[i] = correct_r.cast<float>();
    }
}

void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image, const std_msgs::Header &header)
{


    ///////////////////////////////////////////////////////////////////////
    //TYPE of image = [ < featureID, [<cameraID, Point>, ... ]  >, ....] //
    ///////////////////////////////////////////////////////////////////////
    VINS_LOG("Adding %lu feature points", image.size());
    ///////////////////////////////////////////////////////////////////////
    //MARGIN_OLD Condition:                                              //
    //                    frame_count < 2                                //
    //                    last_track_num < 20                            //
    //                    parallax_num == 0                              //
    //                    parallax_sum / parallax_num >= MIN_PARALLAX    //
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    //     parallax betwwen second last frame and third last frame      //
    ///////////////////////////////////////////////////////////////////////
    if (f_manager.addFeatureCheckParallax(frame_count, image))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    VINS_LOG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    VINS_LOG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    VINS_LOG("Solving %d", frame_count);
    VINS_LOG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ///////////////////////////////////////////////////////////////////////
    //TYPE of image = [ < featureID, [<cameraID, Point>, ... ]  >, ....] //
    ///////////////////////////////////////////////////////////////////////
    ImageFrame imageframe(image, header.stamp.toSec());
    ///////////////////////////////////////////////////////////////////////
    //when frame_count is zero, tmp_pre_intergration is null             //
    //so the very first imageframe.pre_integration is null               //
    ///////////////////////////////////////////////////////////////////////
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    /////////////////////////////////////////////////////////////////////////////////////////
    //tmp_pre_intergration here is initialized for next frame imu  integration             //
    //so acc_0, gyr_0 is the last itume of last imu frame batch                            //
    /////////////////////////////////////////////////////////////////////////////////////////
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)
    {
        /////////////////////////////////////////////////////////////////////////////////////////
        //only frame buffer is full can run into this branch                                   //
        /////////////////////////////////////////////////////////////////////////////////////////
        if (frame_count == WINDOW_SIZE)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;

                VINS_LOG("relative_R : ");
                VINS_LOG("relative_R : %f            %f               %f",  RIC[0](0,0), RIC[0](0,1), RIC[0](0,2));
                VINS_LOG("relative_R : %f            %f               %f",  RIC[0](1,0), RIC[0](1,1), RIC[0](1,2));
                VINS_LOG("relative_R : %f            %f               %f",  RIC[0](2,0), RIC[0](2,1), RIC[0](2,2));
                VINS_LOG("relative_T : %f   %f   %f", TIC[0].transpose()(0), TIC[0].transpose()(1), TIC[0].transpose()(2) );
            }
        }
    }

    VINS_LOG("ESTIMATE_EXTRINSIC = %d ", ESTIMATE_EXTRINSIC);
    VINS_LOG("frame_count = %d ", frame_count);
    VINS_LOG("solver_flag = %d ", solver_flag);

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)//0.1
            {
               result = initialStructure();
               initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {

                solveOdometry();
                VINS_LOG("processImage final cost %f !\n",init_final_cost);
                if(1 && (init_final_cost > 500))//200
                {
                    solver_flag = INITIAL;
                    slideWindow();
                }
                else
                {
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    f_manager.removeFailures();
                    update_loop_correction();
                    last_R = Rs[WINDOW_SIZE];
                    last_P = Ps[WINDOW_SIZE];
                    last_R0 = Rs[0];
                    last_P0 = Ps[0];
                }



                /*********************************
                //////////add by kevin begin//////
                solver_flag = NON_LINEAR;
                solveOdometry();
                slideWindow();
                f_manager.removeFailures();
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                //////////add by kevin end/////////
                **********************************/
            }
            else {
                slideWindow();
            }
        }
        else{
            frame_count++;
        }
    }
    else
    {
        TicToc t_solve;
        solveOdometry();

        if (failureDetection())
        {
            failure_occur = 1;
            clearState();
            setParameter();
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        update_loop_correction();
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        //////////////////////////////////////////////////////////////////////////////
        //here frame_it++ because the first all_image_frame pre_integration is null //
        //////////////////////////////////////////////////////////////////////////////
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        VINS_LOG("IMU excitation var = %f", var);
        if(var < 0.25)
        {
            //VINS_LOG("IMU excitation not enouth!");
            return false;
        }
    }
    // global sfm    //solomon modified for some armeabi-v7a ndk compiler issue
    /*  Quaterniond Q[frame_count + 1];
        Vector3d T[frame_count + 1];
    */
    Quaterniond * Q = new Quaterniond[frame_count + 1];
    boost::shared_array<Quaterniond> QQ(Q);
    Vector3d * T = new Vector3d[frame_count + 1];
    boost::shared_array<Vector3d> TT(T);

    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    //[ <id [<frameId Point> <frameId Point> ...]>   <id [<frameId Point> <frameId Point> ...]>   ......]
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        VINS_LOG("Not enough features or parallax; Move device around");
        return false;
    }

    VINS_LOG("relative_R : ");
    VINS_LOG("relative_R : %f            %f               %f",  relative_R(0,0), relative_R(0,1), relative_R(0,2));
    VINS_LOG("relative_R : %f            %f               %f",  relative_R(1,0), relative_R(1,1), relative_R(1,2));
    VINS_LOG("relative_R : %f            %f               %f",  relative_R(2,0), relative_R(2,1), relative_R(2,2));
    VINS_LOG("relative_T : %f   %f   %f", relative_T.transpose()(0), relative_T.transpose()(1), relative_T.transpose()(2) );
    VINS_LOG("L: %d", l);

    GlobalSFM sfm;
    // output_parameter: Q T sfm_tracked_points
    // so what is the word frame? that's a nice question!
    // the relatice_R and relative_T choose l as the base Frame
    // and the l Camera Frame also become the World Frame!!
    // Q T is the Frame transform of camera
    // all sfm_tracked_points are in l camera frame which is also choosen as the world frame
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        LOGD("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        // set rvec and t an initial guess value for solvePnP
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose(); // frame transform word to imu frame
            frame_it->second.T = T[i];                                         // here ignore the TIC[0] why?
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        /////////////////////////////////////////////////////////////////////////
        ///////   *  map<int, vector<pair<int, Vector3d> > > points;   *    /////
        /////////////////////////////////////////////////////////////////////////
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            VINS_LOG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            VINS_LOG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose(); // frame transform word to imu frame
        frame_it->second.T = T_pnp;                      // here ignore the TIC[0] why?
    }
    if (visualInitialAlign())
        return true;
    else
    {
        VINS_LOG("misalign visual structure with IMU");
        return false;
    }

}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        VINS_LOG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R; // frame transform
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T; // frame transform
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    //for (int i = WINDOW_SIZE-1; i > 0; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        // [0 <--> 10] [1 <--> 10]  [2 <--> 10]  [3 <--> 10] ......
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                LOGD("average_parallax %f choose l %d and newest frame to triangulate the whole structure.corres.size:%d", average_parallax * 460, l, corres.size());
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    //if (solver_flag == NON_LINEAR)
    //{
        TicToc t_tri;

        LOGD("Estimator::solveOdometry::triangulate 0");
        f_manager.triangulate(Ps, tic, ric);
        LOGD("Estimator::solveOdometry::triangulate 1");

        LOGD("Estimator::solveOdometry::optimization 0");
        optimization();
        LOGD("Estimator::solveOdometry::optimization 1");
    //}
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                      para_Pose[0][3],
                                                      para_Pose[0][4],
                                                      para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (LOOP_CLOSURE && relocalize && retrive_data_vector[0].relative_pose && !retrive_data_vector[0].relocalized)
    {
        for (int i = 0; i < (int)retrive_data_vector.size();i++)
            retrive_data_vector[i].relocalized = true;
        Matrix3d vio_loop_r;
        Vector3d vio_loop_t;
        vio_loop_r = rot_diff * Quaterniond(retrive_data_vector[0].loop_pose[6], retrive_data_vector[0].loop_pose[3], retrive_data_vector[0].loop_pose[4], retrive_data_vector[0].loop_pose[5]).normalized().toRotationMatrix();
        vio_loop_t = rot_diff * Vector3d(retrive_data_vector[0].loop_pose[0] - para_Pose[0][0],
                                retrive_data_vector[0].loop_pose[1] - para_Pose[0][1],
                                retrive_data_vector[0].loop_pose[2] - para_Pose[0][2]) + origin_P0;
        Quaterniond vio_loop_q(vio_loop_r);
        double relocalize_yaw;
        relocalize_yaw = Utility::R2ypr(retrive_data_vector[0].R_old).x() - Utility::R2ypr(vio_loop_r).x();
        relocalize_r = Utility::ypr2R(Vector3d(relocalize_yaw, 0, 0));
        relocalize_t = retrive_data_vector[0].P_old- relocalize_r * vio_loop_t;
    }
}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
     //   ROS_INFO(" little feature %d", f_manager.last_track_num);
        return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
     //   ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
   //     ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
     //   ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
    //    ROS_INFO(" big z translation");
        return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
       // ROS_INFO(" big delta_angle ");
        return true;
    }
    return false;
}

int solver_num=0;
double solver_t=0;
void Estimator::optimization()
{
    LOGD(" xxxx Estimator::optimization ceres::Problem ceres::LossFunction 0");
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        //SIZE_POSE  7 compose px py pz, qx qy qz qw
        //SIZE_SPEEDBIAS 9 compose vx vy vz bax bay baz bgx bgy bgz
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            VINS_LOG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            VINS_LOG("estimate extinsic param");
    }

    TicToc t_whole, t_prepare;
    vector2double();

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            f_m_cnt++;
        }
    }

    LOGD("xxxx Estimator::optimization marginalization first 1");

    relocalize = false;
    //loop close factor
    if(LOOP_CLOSURE)
    {
        int loop_constraint_num = 0;
        for (int k = 0; k < (int)retrive_data_vector.size(); k++)
        {
            for(int i = 0; i < WINDOW_SIZE; i++)
            {
                if(retrive_data_vector[k].header == Headers[i].stamp.toSec())
                {
                    relocalize = true;
                    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                    problem.AddParameterBlock(retrive_data_vector[k].loop_pose, SIZE_POSE, local_parameterization);
                    loop_window_index = i;
                    loop_constraint_num++;
                    int retrive_feature_index = 0;
                    int feature_index = -1;
                    for (auto &it_per_id : f_manager.feature)
                    {
                        it_per_id.used_num = it_per_id.feature_per_frame.size();
                        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                            continue;

                        ++feature_index;
                        int start = it_per_id.start_frame;
                        if(start <= i)
                        {
                            while(retrive_data_vector[k].features_ids[retrive_feature_index] < it_per_id.feature_id)
                            {
                                retrive_feature_index++;
                            }

                            if(retrive_data_vector[k].features_ids[retrive_feature_index] == it_per_id.feature_id)
                            {
                                Vector3d pts_j = Vector3d(retrive_data_vector[k].measurements[retrive_feature_index].x, retrive_data_vector[k].measurements[retrive_feature_index].y, 1.0);
                                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                                problem.AddResidualBlock(f, loss_function, para_Pose[start], retrive_data_vector[k].loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);

                                retrive_feature_index++;
                            }
                        }
                    }

                }
            }
        }
    }// END LOOP_CLOSURE

    LOGD("xxxx ceres::Solver::Options 0");

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;

    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    //options.use_nonmonotonic_steps = true;

    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 50 ;// * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME * 50 ; //*1;

//    options.use_explicit_schur_complement = true;
//    options.minimizer_progress_to_stdout = false;
//    options.max_num_iterations = 5;
//    //options.use_nonmonotonic_steps = true;
//    options.max_solver_time_in_seconds = 1;

    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    init_final_cost = summary.final_cost;
    VINS_LOG("tccostlog final cost %f", init_final_cost);
//    cout << summary.BriefReport() << endl;


//    //优化求解起始时间
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//    // 开始优化
//    ceres::Solve ( options, &problem, &summary );
//    //优化求解结束时间
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
//    cout<<"最小二成法优化时间 solve time cost = "<<time_used.count()<<" seconds. "<<endl;

//    LOGD("timelog : 最小二成法优化时间 solve time cost = %d senconds", time_used.count());
//
//    LOGD("timelog : summary.BriefReport = %s", summary.BriefReport());
//    LOGD("timelog : solve Iterations number: %d", static_cast<int>(summary.iterations.size()));
//    LOGDTC("tctimelog : solve time cost: %f", t_solver.toc());

//    if(solver_num < 500)
//    {
//        const double tt = t_solver.toc();
//        solver_t += tt;
//        LOGDTC("tctimelog solver_num: %d  time: %f", solver_num, tt);
//        if(499 == solver_num){
//            double st = solver_t / 500;
//            LOGDTC("tctimelog : solve time %f/%d cost: %f", solver_t,solver_num, st);
//        }
//        solver_num++;
//    }


    LOGD("xxxx ceres::Solver::Options 1");

    // relative info between two loop frame
    if(LOOP_CLOSURE && relocalize)
    { 
        for (int k = 0; k < (int)retrive_data_vector.size(); k++)
        {
            for(int i = 0; i< WINDOW_SIZE; i++)
            {
                if(retrive_data_vector[k].header == Headers[i].stamp.toSec())
                {
                    retrive_data_vector[k].relative_pose = true;
                    Matrix3d Rs_i = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
                    Vector3d Ps_i = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
                    Quaterniond Qs_loop;
                    Qs_loop = Quaterniond(retrive_data_vector[k].loop_pose[6],  retrive_data_vector[k].loop_pose[3],  retrive_data_vector[k].loop_pose[4],  retrive_data_vector[k].loop_pose[5]).normalized().toRotationMatrix();
                    Matrix3d Rs_loop = Qs_loop.toRotationMatrix();
                    Vector3d Ps_loop = Vector3d( retrive_data_vector[k].loop_pose[0],  retrive_data_vector[k].loop_pose[1],  retrive_data_vector[k].loop_pose[2]);

                    retrive_data_vector[k].relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                    retrive_data_vector[k].relative_q = Rs_loop.transpose() * Rs_i;
                    retrive_data_vector[k].relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                    if (abs(retrive_data_vector[k].relative_yaw) > 30.0 || retrive_data_vector[k].relative_t.norm() > 20.0)
                        retrive_data_vector[k].relative_pose = false;
                        
                }
            } 
        } 
    }

    double2vector();

    LOGD("xxxx Estimator::optimization marginalization second 0");

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
//    if(0)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                   vector<int>{0, 3});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
       // LOGD("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
     //   LOGD("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
		            assert(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
        //    LOGD("begin marginalization");
            marginalization_info->preMarginalize();
         //   LOGD("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
         //   LOGD("begin marginalization");
            marginalization_info->marginalize();
         //   LOGD("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }

    LOGD("xxxx Estimator::optimization marginalization second 1");
  //  LOGD("whole marginalization costs: %f", t_whole_marginalization.toc());
   
   // LOGD("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++) // i = [ 0 ... 9]
            {
                Rs[i].swap(Rs[i + 1]);   // [0<->1] ....... [9<->10]

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                double t_0 = Headers[0].stamp.toSec();
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);

            }

            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];  // frame imu to camera Rotation
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0]; // frame imu to camera Translation
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();

}

