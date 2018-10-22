#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
//template <typename T>
//void reduceVector(vector<T> &v, vector<uchar> status)
//{
//    int j = 0;
//    for (int i = 0; i < int(v.size()); i++)
//        if (status[i])
//            v[j++] = v[i];
//    v.resize(j);
//}

FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
    //---------------------------------------------------------------------------//
    // 0 for black color, and black area is disabled for searching feature points//
    //---------------------------------------------------------------------------//
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    //---------------------------------------------------------------------------//
    //       prefer to keep features that are tracked for long time              //
    //---------------------------------------------------------------------------//
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    //vector<pair<pair<int, max_min_pts>, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));
        //cnt_pts_id.push_back(make_pair(make_pair(track_cnt[i], parallax_cnt[i]), make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
    {
        return a.first > b.first;
    });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    parallax_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            //parallax_cnt.push_back(it.first.second);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, vector<Point2f> &good_pts, vector<int> &track_len)
{
    cv::Mat img;
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
    }
    else{
        img = _img;
    }

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
    }
    VINS_LOG("readImage PUB_THIS_FRAME before");
    if (PUB_THIS_FRAME)
    {
        VINS_LOG("readImage PUB_THIS_FRAME body %d",track_cnt.size());
        rejectWithF();

        for (auto &n : track_cnt)
            n++;

        TicToc t_m;
        //------------------------------------------------------------------//
        //    in setMask(), we also sort the forw_pts, ids and track_cnt     //
        //                  in the track_cnt decrease order                 //
        //------------------------------------------------------------------//
        setMask();

        TicToc t_t;
        //------------------------------------------------------------------//
        //      by now MAX_CNT is set to 70, maybe modified later           //
        //------------------------------------------------------------------//
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());

        if (n_max_cnt > 0)
        {
            if(mask.empty()){
                VINS_LOG("readImage mask is empty");
            }

            if (mask.type() != CV_8UC1){
                VINS_LOG("readImage mask type wrong");
            }

            if (mask.size() != forw_img.size()) {
                VINS_LOG("readImage wrong size");
            }
            n_pts.clear();
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.1, MIN_DIST, mask);
        }
        else
            n_pts.clear();

        TicToc t_a;
        addPoints();

        prev_img = forw_img;
        prev_pts = forw_pts;
    }
    //------------------------------------------------------------------//
    //         prev_img is the pre image in last PUBLISH                //
    //         cur_img  is the pre image in last process                //
    //------------------------------------------------------------------//
    //         prev_pts is used for projecting bad points               //
    //         cur_pts  is used for tracing                             //
    //------------------------------------------------------------------//
    //   prev_pts  cur_pts  are all in the image coordinate space       //
    //------------------------------------------------------------------//
    cur_img = forw_img;
    cur_pts = forw_pts;
    good_pts = cur_pts;
    track_len = track_cnt;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        //VINS_LOG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_prev_pts(prev_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        //------------------------------------------------------------------//
        // un_prev_pts  un_forw_pts  are all in the image coordinate space  //
        //------------------------------------------------------------------//
        cv::findFundamentalMat(un_prev_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
         //reduceVector(parallax_cnt, status);
         //VINS_LOG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
         //VINS_LOG("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    VINS_LOG("[INFO] FeatureTracker: generateCameraFromYamlFile");
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPoints()
{
    vector<cv::Point2f> un_pts;
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }

    return un_pts;
}
