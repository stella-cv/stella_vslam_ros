#include <openvslam_ros.h>
#include <openvslam/publish/map_publisher.h>

#include <chrono>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>

namespace openvslam_ros {
system::system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : SLAM_(cfg, vocab_file_path), cfg_(cfg), node_(std::make_shared<rclcpp::Node>("run_slam")), custom_qos_(rmw_qos_profile_default),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)),
      pose_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("~/camera_pose", 1)),
      map_to_odom_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
      tf_(std::make_unique<tf2_ros::Buffer>(node_->get_clock())),
      transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_)) {
    custom_qos_.depth = 1;
    exec_.add_node(node_);
    init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&system::init_pose_callback,
                  this, std::placeholders::_1));
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp) {
    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
    Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
    Eigen::Affine3d map_to_camera_affine(trans * rot);
    Eigen::Matrix3d rot_ros_to_cv_map_frame;
    rot_ros_to_cv_map_frame << 0, 0, 1,
        -1, 0, 0,
        0, -1, 0;

    // Transform map frame from CV coordinate system to ROS coordinate system
    map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame);

    // Create odometry message and update it with current camera pose
    nav_msgs::msg::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_link_;
    pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine);
    pose_pub_->publish(pose_msg);

    // Send map->odom transform. Set publish_tf to false if not using TF
    if (publish_tf_) {
        try {
            auto camera_to_odom = tf_->lookupTransform(
                camera_link_, odom_frame_, tf2_ros::fromMsg(builtin_interfaces::msg::Time(stamp)),
                tf2::durationFromSec(0.0));
            Eigen::Affine3d camera_to_odom_affine = tf2::transformToEigen(camera_to_odom.transform);

            auto map_to_odom_msg = tf2::eigenToTransform(map_to_camera_affine * camera_to_odom_affine);
            tf2::TimePoint transform_timestamp = tf2_ros::fromMsg(stamp) + tf2::durationFromSec(transform_tolerance_);
            map_to_odom_msg.header.stamp = tf2_ros::toMsg(transform_timestamp);
            map_to_odom_msg.header.frame_id = map_frame_;
            map_to_odom_msg.child_frame_id = odom_frame_;
            map_to_odom_broadcaster_->sendTransform(map_to_odom_msg);
        }
        catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Transform failed: %s", ex.what());
        }
    }
}

void system::setParams() {
    odom_frame_ = std::string("odom");
    odom_frame_ = node_->declare_parameter("odom_frame", odom_frame_);

    map_frame_ = std::string("map");
    map_frame_ = node_->declare_parameter("map_frame", map_frame_);

    // Set publish_tf to false if not using TF
    publish_tf_ = true;
    publish_tf_ = node_->declare_parameter("publish_tf", publish_tf_);

    // Publish pose's timestamp in the future
    transform_tolerance_ = 0.5;
    transform_tolerance_ = node_->declare_parameter("transform_tolerance", transform_tolerance_);
}

void system::init_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    Eigen::Matrix4d cam_pose_ros(Eigen::Matrix4d::Identity());
    Eigen::Vector3d pose_v(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    cam_pose_ros.block<3, 1>(0, 3) = pose_v;

    Eigen::Quaterniond rot_q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    Eigen::Matrix3d rot_m = rot_q.toRotationMatrix();
    cam_pose_ros.block<3, 3>(0, 0) = rot_m;

    Eigen::Matrix4d rot_ros_to_cv_map_frame;
    rot_ros_to_cv_map_frame << 0, 0, 1, 0,
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 0, 1;

    // Transforming initial pose from ROS coordinate system to CV coordinate system as follows:
    // T(map, pose_cv) = T(map, pose) * T(pose, pose_cv)
    // T(map, pose_cv) = T(map, map_cv) * T(cv, pose_cv)
    // where T(pose, pose_cv) == T(map, map_cv)
    // ||
    // \/
    // T(cv, pose_cv) = T(map, map_cv).inv * T(map, pose) * T(map, map_cv)
    Eigen::Matrix4d cam_pose_cv = rot_ros_to_cv_map_frame.transpose() * cam_pose_ros * rot_ros_to_cv_map_frame;

    if (!SLAM_.update_pose(cam_pose_cv)) {
        RCLCPP_ERROR(node_->get_logger(), "Can not set initial pose");
    }
}

mono::mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
    sub_ = image_transport::create_subscription(
        node_.get(), "camera/image_raw", [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { callback(msg); }, "raw", custom_qos_);
}
void mono::callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (camera_link_.empty()) {
        camera_link_ = msg->header.frame_id;
    }
    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = tp_1.seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    //track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, msg->header.stamp);
    }
}

stereo::stereo(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
               const bool rectify)
    : system(cfg, vocab_file_path, mask_img_path),
      rectifier_(rectify ? std::make_shared<openvslam::util::stereo_rectifier>(cfg) : nullptr),
      left_sf_(node_, "camera/left/image_raw"),
      right_sf_(node_, "camera/right/image_raw"),
      sync_(left_sf_, right_sf_, 10) {
    sync_.registerCallback(&stereo::callback, this);
}

void stereo::callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right) {
    if (camera_link_.empty()) {
        camera_link_ = left->header.frame_id;
    }
    auto leftcv = cv_bridge::toCvShare(left)->image;
    auto rightcv = cv_bridge::toCvShare(right)->image;
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    if (rectifier_) {
        rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
    }

    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = tp_1.seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    //track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, left->header.stamp);
    }
}

rgbd::rgbd(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path),
      color_sf_(node_, "camera/color/image_raw"),
      depth_sf_(node_, "camera/depth/image_raw"),
      sync_(color_sf_, depth_sf_, 10) {
    sync_.registerCallback(&rgbd::callback, this);
}

void rgbd::callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    if (camera_link_.empty()) {
        camera_link_ = color->header.frame_id;
    }
    auto colorcv = cv_bridge::toCvShare(color)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }

    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = tp_1.seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track time in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, color->header.stamp);
    }
}

} // namespace openvslam_ros
