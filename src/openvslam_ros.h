#ifndef OPENVSLAM_ROS_H
#define OPENVSLAM_ROS_H

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/util/stereo_rectifier.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace openvslam_ros {
class system {
public:
    system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    void publish_pose(const Eigen::Matrix4d& cam_pose_wc, const ros::Time& stamp);
    void setParams();
    openvslam::system SLAM_;
    std::shared_ptr<openvslam::config> cfg_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    std::chrono::steady_clock::time_point tp_0_;
    cv::Mat mask_;
    std::vector<double> track_times_;
    ros::Publisher pose_pub_;
    ros::Subscriber init_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    std::string odom_frame_, map_frame_, base_link_, camera_link_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    bool publish_tf_;
    double transform_tolerance_;

private:
    void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
};

class mono : public system {
public:
    mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    void callback(const sensor_msgs::ImageConstPtr& msg);

    image_transport::Subscriber sub_;
};

class stereo : public system {
public:
    stereo(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
           const bool rectify);
    void callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right);

    std::shared_ptr<openvslam::util::stereo_rectifier> rectifier_;
    image_transport::SubscriberFilter left_sf_, right_sf_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
    message_filters::Synchronizer<SyncPolicy> sync_;
};

class rgbd : public system {
public:
    rgbd(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    void callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth);

    image_transport::SubscriberFilter color_sf_, depth_sf_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
    message_filters::Synchronizer<SyncPolicy> sync_;
};
} // namespace openvslam_ros

#endif // OPENVSLAM_ROS_H
