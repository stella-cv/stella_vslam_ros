#include <openvslam_ros.h>

#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <openvslam/publish/map_publisher.h>
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
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc) {
    const std::lock_guard<std::mutex> lock(camera_link_mutex);
    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rot = cam_pose_wc.block<3, 3>(0, 0);
    Eigen::Vector3d trans = cam_pose_wc.block<3, 1>(0, 3);
    Eigen::Matrix3d cv_to_ros;
    cv_to_ros << 0, 0, 1,
        -1, 0, 0,
        0, -1, 0;

    // Transform from CV coordinate system to ROS coordinate system on camera coordinates
    Eigen::Quaterniond quat(cv_to_ros * rot * cv_to_ros.transpose());
    trans = cv_to_ros * trans;

    // Create odometry message and update it with current camera pose
    nav_msgs::msg::Odometry pose_msg;
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_link_;
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();
    pose_msg.pose.pose.position.x = trans(0);
    pose_msg.pose.pose.position.y = trans(1);
    pose_msg.pose.pose.position.z = trans(2);
    pose_pub_->publish(pose_msg);

    // Send map->odom transform. Set publish_tf_ to false with not using TF
    if (publish_tf_) {
        tf2::Stamped<tf2::Transform> camera_to_map(tf2::Transform(tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()),
                                                                  tf2::Vector3(trans(0), trans(1), trans(2)))
                                                       .inverse(),
                                                   tf2_ros::fromMsg(node_->now()), camera_link_);

        geometry_msgs::msg::TransformStamped camera_to_map_msg, odom_to_map_msg, map_to_odom_msg;
        tf2::Stamped<tf2::Transform> odom_to_map_stamped;

        camera_to_map_msg = tf2::toMsg(camera_to_map);

        try {
            odom_to_map_msg = tf_->transform(camera_to_map_msg, odom_frame_);
            tf2::fromMsg(odom_to_map_msg, odom_to_map_stamped);

            map_to_odom_msg.transform = tf2::toMsg(tf2::Transform(tf2::Quaternion(odom_to_map_stamped.getRotation()),
                                                                  tf2::Vector3(odom_to_map_stamped.getOrigin()))
                                                       .inverse());
            map_to_odom_msg.header.frame_id = map_frame_;
            map_to_odom_msg.child_frame_id = odom_frame_;
            map_to_odom_msg.header.stamp = node_->now();
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

    publish_tf_ = true;
    publish_tf_ = node_->declare_parameter("publish_tf_", publish_tf_);
}

mono::mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
    sub_ = image_transport::create_subscription(
        node_.get(), "camera/image_raw", [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { callback(msg); }, "raw", custom_qos_);
}
void mono::callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (camera_link_.empty()) {
        const std::lock_guard<std::mutex> lock(camera_link_mutex);
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
        publish_pose(*cam_pose_wc);
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
        const std::lock_guard<std::mutex> lock(camera_link_mutex);
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
        publish_pose(*cam_pose_wc);
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
        const std::lock_guard<std::mutex> lock(camera_link_mutex);
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
        publish_pose(*cam_pose_wc);
    }
}

} // namespace openvslam_ros
