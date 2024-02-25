#ifndef STELLA_SLAM_ROS_H
#define STELLA_SLAM_ROS_H

#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/stereo_rectifier.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace stella_vslam_ros {
class system {
public:
    system(const std::shared_ptr<stella_vslam::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path);
    void publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp);
    void publish_keyframes(const rclcpp::Time& stamp);
    void setParams();
    std::shared_ptr<stella_vslam::system> slam_;
    std::shared_ptr<stella_vslam::config> cfg_;
    rclcpp::Node* node_;
    rmw_qos_profile_t custom_qos_;
    cv::Mat mask_;
    std::vector<double> track_times_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> keyframes_pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> keyframes_2d_pub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
        init_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string robot_base_frame_;
    std::string camera_frame_;
    std::string camera_optical_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    // If true, publish tf from map_frame to odom_frame
    bool publish_tf_;

    // If true, publish keyframes
    bool publish_keyframes_;

    // Publish pose's timestamp in the future
    double transform_tolerance_;

    // If true, odom_frame is fixed on the xy-plane of map_frame. This is useful when working with 2D navigation modules.
    bool odom2d_;

    std::string encoding_;

private:
    void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    Eigen::AngleAxisd rot_ros_to_cv_map_frame_;
};

class mono : public system {
public:
    mono(const std::shared_ptr<stella_vslam::system>& slam,
         rclcpp::Node* node,
         const std::string& mask_img_path);
    void callback(sensor_msgs::msg::Image::UniquePtr msg);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> raw_image_sub_;
};

template<class M, class NodeType = rclcpp::Node>
class ModifiedSubscriber : public message_filters::Subscriber<M> {
public:
    ModifiedSubscriber(typename message_filters::Subscriber<M>::NodePtr node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
        : message_filters::Subscriber<M>(node, topic, qos) {
    }
    ModifiedSubscriber(NodeType* node, const std::string& topic, const rmw_qos_profile_t qos = rmw_qos_profile_default)
        : message_filters::Subscriber<M>(node, topic, qos) {
    }
    ModifiedSubscriber(
        typename message_filters::Subscriber<M>::NodePtr node,
        const std::string& topic,
        const rmw_qos_profile_t qos,
        rclcpp::SubscriptionOptions options)
        : message_filters::Subscriber<M>(node, topic, qos, options) {
    }
    ModifiedSubscriber(
        NodeType* node,
        const std::string& topic,
        const rmw_qos_profile_t qos,
        rclcpp::SubscriptionOptions options)
        : message_filters::Subscriber<M>(node, topic, qos, options) {
    }
    void cb(const typename message_filters::Subscriber<M>::MConstPtr& msg) {
        this->signalMessage(msg);
    }
};

class stereo : public system {
public:
    stereo(const std::shared_ptr<stella_vslam::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path,
           const std::shared_ptr<stella_vslam::util::stereo_rectifier>& rectifier);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right);

    std::shared_ptr<stella_vslam::util::stereo_rectifier> rectifier_;
    ModifiedSubscriber<sensor_msgs::msg::Image> left_sf_, right_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

class rgbd : public system {
public:
    rgbd(const std::shared_ptr<stella_vslam::system>& slam,
         rclcpp::Node* node,
         const std::string& mask_img_path);
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth);

    ModifiedSubscriber<sensor_msgs::msg::Image> color_sf_, depth_sf_;
    using ApproximateTimeSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
    using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
    bool use_exact_time_;
};

} // namespace stella_vslam_ros

#endif // STELLA_SLAM_ROS_H
