#ifndef OPENVSLAM_ROS_H
#define OPENVSLAM_ROS_H

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/util/stereo_rectifier.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace openvslam_ros {
class system {
public:
    system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    void publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp);
    void setParams();
    openvslam::system SLAM_;
    std::shared_ptr<openvslam::config> cfg_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    rmw_qos_profile_t custom_qos_;
    cv::Mat mask_;
    std::vector<double> track_times_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
        init_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    std::string odom_frame_, map_frame_, base_link_;
    std::string camera_frame_, camera_optical_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    bool publish_tf_;
    double transform_tolerance_;

private:
    void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};

class mono : public system {
public:
    mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    image_transport::Subscriber sub_;
};

class stereo : public system {
public:
    stereo(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
           const bool rectify);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right);

    std::shared_ptr<openvslam::util::stereo_rectifier> rectifier_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sf_, right_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

class rgbd : public system {
public:
    rgbd(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth);

    message_filters::Subscriber<sensor_msgs::msg::Image> color_sf_, depth_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

} // namespace openvslam_ros

#endif // OPENVSLAM_ROS_H
