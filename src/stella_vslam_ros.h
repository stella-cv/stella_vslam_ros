#ifndef STELLA_SLAM_ROS_H
#define STELLA_SLAM_ROS_H

#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/stereo_rectifier.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>

namespace stella_vslam_ros {
class system {
public:
    system(const std::shared_ptr<stella_vslam::system>& slam,
           const std::string& mask_img_path);
    void publish_pose(const Eigen::Matrix4d& cam_pose_wc, const ros::Time& stamp);
    void publish_pointcloud(const ros::Time& stamp);
    void publish_keyframes(const ros::Time& stamp);
    void setParams();

    std::shared_ptr<stella_vslam::system> slam_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    std::chrono::steady_clock::time_point tp_0_;
    cv::Mat mask_;
    std::vector<double> track_times_;
    ros::Publisher pose_pub_;
    ros::Publisher pc_pub_;
    ros::Publisher keyframes_pub_;
    ros::Publisher keyframes_2d_pub_;
    ros::Subscriber init_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    std::string odom_frame_, map_frame_, base_link_;
    std::string camera_frame_, camera_optical_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    bool publish_tf_;
    bool publish_pointcloud_;
    bool publish_keyframes_;
    double transform_tolerance_;

private:
    void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    Eigen::AngleAxisd rot_ros_to_cv_map_frame_;
};

class mono : public system {
public:
    mono(const std::shared_ptr<stella_vslam::system>& slam,
         const std::string& mask_img_path);
    void callback(const sensor_msgs::ImageConstPtr& msg);

    image_transport::Subscriber sub_;
};

class stereo : public system {
public:
    stereo(const std::shared_ptr<stella_vslam::system>& slam,
           const std::string& mask_img_path,
           const std::shared_ptr<stella_vslam::util::stereo_rectifier>& rectifier);
    void callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right);

    std::shared_ptr<stella_vslam::util::stereo_rectifier> rectifier_;
    image_transport::SubscriberFilter left_sf_, right_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

class rgbd : public system {
public:
    rgbd(const std::shared_ptr<stella_vslam::system>& slam,
         const std::string& mask_img_path);
    void callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth);

    image_transport::SubscriberFilter color_sf_, depth_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};
} // namespace stella_vslam_ros

#endif // STELLA_SLAM_ROS_H
