/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo_inertial.cc
*
*/

#include "common.h"

using namespace std;

bool whether_publish_tf_transform;
bool interpolation = false;
float interpolation_delay = 0.2;
int interpolation_sample_num = 4;
int interpolation_order = 2;

bool updated = false;
ros::Time time_ref(0.0);

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
    void publish();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    std::deque<geometry_msgs::PoseStamped> pose_msgs;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/pose_frame_id", pose_frame_id, "pose");

    bool bEqual = false, do_rectify = false;
    node_handler.param<bool>(node_name + "/do_equalize", bEqual, false);
    node_handler.param<bool>(node_name + "/do_rectify", do_rectify, false);

    bool bUseViewer = false;
    node_handler.param<bool>(node_name + "/use_viewer", bUseViewer, false);
    
    node_handler.param<bool>(node_name + "/publish_tf_transform", whether_publish_tf_transform, true);
    
    // interpolation
    node_handler.param<bool>(node_name + "/interpolation", interpolation, false);
    node_handler.param<float>(node_name + "/interpolation_delay", interpolation_delay, 0.2);
    node_handler.param<int>(node_name + "/interpolation_order", interpolation_order, 2);
    node_handler.param<int>(node_name + "/interpolation_sample_num", interpolation_sample_num, 4);

    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::IMU_STEREO, bUseViewer);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, do_rectify, bEqual);
    
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(settings_file, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    // Maximum delay, 5 seconds * 200Hz = 1000 samples
    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img_left = node_handler.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = node_handler.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);

    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped> ("/orb_slam3_ros/camera", 1);
    map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3_ros/map_points", 1);

    setup_tf_orb_to_ros(ORB_SLAM3::System::IMU_STEREO);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb); 

    // std::thread sync_thread_pub(&ImageGrabber::publish, &igb); 

    ros::spin();

    return 0;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
        {
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
        tImRight = imgRightBuf.front()->header.stamp.toSec();

        this->mBufMutexRight.lock();
        while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
        {
            imgRightBuf.pop();
            tImRight = imgRightBuf.front()->header.stamp.toSec();
        }
        this->mBufMutexRight.unlock();

        this->mBufMutexLeft.lock();
        while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
        {
            imgLeftBuf.pop();
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
        }
        this->mBufMutexLeft.unlock();

        if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
        {
            // std::cout << "big time difference" << std::endl;
            continue;
        }
        if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
            continue;

        this->mBufMutexLeft.lock();
        imLeft = GetImage(imgLeftBuf.front());
        ros::Time current_frame_time = imgLeftBuf.front()->header.stamp;
        imgLeftBuf.pop();
        this->mBufMutexLeft.unlock();

        this->mBufMutexRight.lock();
        imRight = GetImage(imgRightBuf.front());
        imgRightBuf.pop();
        this->mBufMutexRight.unlock();

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        mpImuGb->mBufMutex.lock();
        if(!mpImuGb->imuBuf.empty())
        {
            // Load imu measurements from buffer
            vImuMeas.clear();
            while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
            {
            double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
            cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
            cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
            mpImuGb->imuBuf.pop();
            }
        }
        mpImuGb->mBufMutex.unlock();
        if(mbClahe)
        {
            mClahe->apply(imLeft,imLeft);
            mClahe->apply(imRight,imRight);
        }

        if(do_rectify)
        {
            cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
            cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
        }
        
        // Main algorithm runs here
        cv::Mat Tcw = mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);

        if (!Tcw.empty())
        {
            tf::Transform tf_transform = from_orb_to_ros_tf_transform (Tcw);

            if(whether_publish_tf_transform) publish_tf_transform(tf_transform, current_frame_time);

            tf::Stamped<tf::Pose> grasp_tf_pose(tf_transform, current_frame_time, map_frame_id);

            geometry_msgs::PoseStamped pose_msg;

            tf::poseStampedTFToMsg(grasp_tf_pose, pose_msg);
            
            // pose_msgs.push_back(pose_msg);
            pose_pub.publish(pose_msg);
            updated = true;
        }

        publish_ros_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), current_frame_time);

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
        }
    }
}

void ImageGrabber::publish()
{
    while(1)
    {
        if( true)
        {
            if(!pose_msgs.empty())
            {
                geometry_msgs::PoseStamped vision = pose_msgs.front();
                pose_msgs.pop_front();
                pose_pub.publish(vision);
            }
        }
        else
        {
            while(pose_msgs.size() > interpolation_sample_num)
            {
                pose_msgs.pop_front();
            }
            if(pose_msgs.size() == interpolation_sample_num)
            {
                static Eigen::MatrixXf A(interpolation_sample_num,interpolation_order+1);
                static Eigen::MatrixXf b(interpolation_sample_num,7);
                static Eigen::MatrixXf X(interpolation_order+1,7);
                static ros::Time time_stamp_header;

                if(time_ref.toSec() == 0.0 || updated)
                {
                    // Fitting
                    time_ref = pose_msgs.front().header.stamp;
                    std::deque<geometry_msgs::PoseStamped>::iterator it = pose_msgs.begin();
                    int idx=0;
                    while(it != pose_msgs.end()){
                        float t = (it->header.stamp-time_ref).toSec();

                        A(idx,0) = 1;
                        for(int i=1;i<=interpolation_order;i++)
                        {
                            A(idx,i) = t*A(idx,i-1);
                        }
                        b(idx,0) = it->pose.position.x;
                        b(idx,1) = it->pose.position.y;
                        b(idx,2) = it->pose.position.z;
                        b(idx,3) = it->pose.orientation.x;
                        b(idx,4) = it->pose.orientation.y;
                        b(idx,5) = it->pose.orientation.z;
                        b(idx,6) = it->pose.orientation.w;
                        it++;
                        idx++;
                    }
                    X = (A.transpose() * A).llt().solve(A.transpose() * b);
                }

                time_stamp_header = ros::Time::now() - ros::Duration(interpolation_delay);

                if(pose_msgs.back().header.stamp >= time_stamp_header){
                    // Interpolation
                    geometry_msgs::PoseStamped vision;
                    vision.header.stamp = time_stamp_header + ros::Duration(interpolation_delay);// default delay
                    vision.header.frame_id = "/world";
                    float t = (time_stamp_header-time_ref).toSec();

                    Eigen::VectorXf T(interpolation_order+1);
                    T(0) = 1;
                    for(int i=1;i<=interpolation_order;i++)
                    {
                        T(i) = t*T(i-1);
                    }
            
                    // Publish
                    Eigen::VectorXf state(7);
                    state = X.transpose()*T;

                    vision.pose.position.x = state[0];
                    vision.pose.position.y = state[1];
                    vision.pose.position.z = state[2];

                    Eigen::Quaternion<double> q = {state[6],state[3],state[4],state[5]};
                    q.normalize();
                    vision.pose.orientation.x = q.x();
                    vision.pose.orientation.y = q.y();
                    vision.pose.orientation.z = q.z();
                    vision.pose.orientation.w = q.w();
                    
                    pose_pub.publish(vision);
                }
                updated = false;
            }
        }
        
        std::chrono::milliseconds tSleep(20);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}
