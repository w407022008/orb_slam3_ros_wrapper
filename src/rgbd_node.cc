/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
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

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void publish();

    ORB_SLAM3::System* mpSLAM;

    std::deque<geometry_msgs::PoseStamped> pose_msgs;
    std::mutex mBufMutexPose;

    ros::Time time_stamp_header;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
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
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::RGBD, bUseViewer);

    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/camera/depth_registered/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped> ("/orb_slam3_ros/camera", 1);

    map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3_ros/map_points", 1);

    setup_tf_orb_to_ros(ORB_SLAM3::System::RGBD);

    std::thread sync_thread_pub(&ImageGrabber::publish, &igb); 

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Main algorithm runs here
    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());

    ros::Time current_frame_time = cv_ptrRGB->header.stamp;

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
