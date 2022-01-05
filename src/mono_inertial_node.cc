/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
*
*/

#include "common.h"
#include <deque>
#include <Eigen/Dense>

using namespace std;

bool whether_publish_tf_transform;
bool interpolation = false;
float interpolation_delay = 0.2;
int interpolation_sample_num = 4;
int interpolation_order = 2;

Eigen::MatrixXf A(interpolation_sample_num,interpolation_order+1);
Eigen::MatrixXf b(interpolation_sample_num,7);
Eigen::MatrixXf X(interpolation_order+1,7);

bool updated = false;
ros::Time time_ref;

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
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
    void publish();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    std::deque<geometry_msgs::PoseStamped> pose_msgs;
    std::mutex mBufMutexPose;

    ros::Time time_stamp_header;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();
    image_transport::ImageTransport image_transport(node_handler);

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

    bool bEqual = false;
    node_handler.param<bool>(node_name + "/do_equalize", bEqual, false);
    bool bUseViewer = false;
    node_handler.param<bool>(node_name + "/use_viewer", bUseViewer, false);
    
    node_handler.param<bool>(node_name + "/publish_tf_transform", whether_publish_tf_transform, false);
    
    // interpolation
    node_handler.param<bool>(node_name + "/interpolation", interpolation, true);
    node_handler.param<float>(node_name + "/interpolation_delay", interpolation_delay, 0.2);
    node_handler.param<int>(node_name + "/interpolation_order", interpolation_order, 2);
    node_handler.param<int>(node_name + "/interpolation_sample_num", interpolation_sample_num, 4);

    A.resize(interpolation_sample_num,interpolation_order+1);
    b.resize(interpolation_sample_num,7);
    X.resize(interpolation_order+1,7);
    A.setZero();
    b.setZero();
    X.setZero();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, bUseViewer);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, bEqual);

    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img0 = node_handler.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &igb);

    setup_ros_publishers(node_handler, image_transport);

    setup_tf_orb_to_ros(ORB_SLAM3::System::IMU_MONOCULAR);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    std::thread sync_thread_pub(&ImageGrabber::publish, &igb); 

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
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
    while(1)
    {
        if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
        {
            cv::Mat im;
            double tIm = 0;
            ros::Time current_frame_time;

            tIm = img0Buf.front()->header.stamp.toSec();
            if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;
            
            {
            this->mBufMutex.lock();
            im = GetImage(img0Buf.front());
            current_frame_time = img0Buf.front()->header.stamp;
            img0Buf.pop();
            this->mBufMutex.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if(mbClahe)
                mClahe->apply(im,im);

            // Main algorithm runs here
            cv::Mat Tcw = mpSLAM->TrackMonocular(im, tIm, vImuMeas);

            if (!Tcw.empty())
            {
                tf::Transform tf_transform = from_orb_to_ros_tf_transform (Tcw);

                if(whether_publish_tf_transform) publish_tf_transform(tf_transform, current_frame_time);

                tf::Stamped<tf::Pose> grasp_tf_pose(tf_transform, current_frame_time, map_frame_id);

                geometry_msgs::PoseStamped pose_msg;

                tf::poseStampedTFToMsg(grasp_tf_pose, pose_msg);
                
                this->mBufMutexPose.lock();
                pose_msgs.push_back(pose_msg);
                updated = true;
                this->mBufMutexPose.unlock();
                // pose_pub.publish(pose_msg);
            }

            publish_ros_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), current_frame_time);

            // publish_ros_tracking_img(mpSLAM->GetCurrentFrame(), current_frame_time);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImageGrabber::publish()
{
    while(1)
    {
        mBufMutexPose.lock();
        if(!interpolation)
        {
            if(!pose_msgs.empty())
            {
                geometry_msgs::PoseStamped vision = pose_msgs.back();
                pose_msgs.clear();
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
                if(updated)
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
                    auto res = A.transpose() * A;
                    auto val = A.transpose() * b;
                    auto  inv = res.inverse();
                    X = res.llt().solve(val);
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
        mBufMutexPose.unlock();
        std::chrono::milliseconds tSleep(10);
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
