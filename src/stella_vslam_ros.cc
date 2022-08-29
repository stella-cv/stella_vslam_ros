#include <stella_vslam_ros.h>
#include <stella_vslam/publish/map_publisher.h>

#include <chrono>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>

namespace stella_vslam_ros {
system::system(const std::shared_ptr<stella_vslam::system>& slam,
               const std::string& mask_img_path)
    : slam_(slam), node_(std::make_shared<rclcpp::Node>("run_slam")), custom_qos_(rmw_qos_profile_default),
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
    setParams();
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp) {
    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
    Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
    Eigen::Affine3d map_to_camera_affine(trans * rot);
    Eigen::AngleAxisd rot_ros_to_cv_map_frame(
        (Eigen::Matrix3d() << 0, 0, 1,
         -1, 0, 0,
         0, -1, 0)
            .finished());

    // Transform map frame from CV coordinate system to ROS coordinate system
    map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame);

    // Create odometry message and update it with current camera pose
    nav_msgs::msg::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_frame_;
    pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine * rot_ros_to_cv_map_frame.inverse());
    pose_pub_->publish(pose_msg);

    // Send map->odom transform. Set publish_tf to false if not using TF
    if (publish_tf_) {
        try {
            auto camera_to_odom = tf_->lookupTransform(
                camera_optical_frame_, odom_frame_, tf2_ros::fromMsg(builtin_interfaces::msg::Time(stamp)),
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
            RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        }
    }
}

void system::setParams() {
    odom_frame_ = std::string("odom");
    odom_frame_ = node_->declare_parameter("odom_frame", odom_frame_);

    map_frame_ = std::string("map");
    map_frame_ = node_->declare_parameter("map_frame", map_frame_);

    base_link_ = std::string("base_footprint");
    base_link_ = node_->declare_parameter("base_link", base_link_);

    camera_frame_ = std::string("camera_frame");
    camera_frame_ = node_->declare_parameter("camera_frame", camera_frame_);

    // Set publish_tf to false if not using TF
    publish_tf_ = true;
    publish_tf_ = node_->declare_parameter("publish_tf", publish_tf_);

    // Publish pose's timestamp in the future
    transform_tolerance_ = 0.5;
    transform_tolerance_ = node_->declare_parameter("transform_tolerance", transform_tolerance_);
}

void system::init_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (camera_optical_frame_.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Camera link is not set: no images were received yet");
        return;
    }

    Eigen::Translation3d trans(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    Eigen::Quaterniond rot_q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    Eigen::Affine3d initialpose_affine(trans * rot_q);

    Eigen::Matrix3d rot_cv_to_ros_map_frame;
    rot_cv_to_ros_map_frame << 0, -1, 0,
        0, 0, -1,
        1, 0, 0;

    Eigen::Affine3d map_to_initialpose_frame_affine;
    try {
        auto map_to_initialpose_frame = tf_->lookupTransform(
            map_frame_, msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        map_to_initialpose_frame_affine = tf2::transformToEigen(
            map_to_initialpose_frame.transform);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    Eigen::Affine3d base_link_to_camera_affine;
    try {
        auto base_link_to_camera = tf_->lookupTransform(
            base_link_, camera_optical_frame_, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        base_link_to_camera_affine = tf2::transformToEigen(base_link_to_camera.transform);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    // Target transform is map_cv -> camera_link and known parameters are following:
    //   rot_cv_to_ros_map_frame: T(map_cv -> map)
    //   map_to_initialpose_frame_affine: T(map -> `msg->header.frame_id`)
    //   initialpose_affine: T(`msg->header.frame_id` -> base_link)
    //   base_link_to_camera_affine: T(base_link -> camera_link)
    // The flow of the transformation is as follows:
    //   map_cv -> map -> `msg->header.frame_id` -> base_link -> camera_link
    Eigen::Matrix4d cam_pose_cv = (rot_cv_to_ros_map_frame * map_to_initialpose_frame_affine
                                   * initialpose_affine * base_link_to_camera_affine)
                                      .matrix();

    const Eigen::Vector3d normal_vector = (Eigen::Vector3d() << 0., 1., 0.).finished();
    if (!slam_->relocalize_by_pose_2d(cam_pose_cv, normal_vector)) {
        RCLCPP_ERROR(node_->get_logger(), "Can not set initial pose");
    }
}

mono::mono(const std::shared_ptr<stella_vslam::system>& slam,
           const std::string& mask_img_path)
    : system(slam, mask_img_path) {
    sub_ = image_transport::create_subscription(
        node_.get(), "camera/image_raw", [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { callback(msg); }, "raw", custom_qos_);
}
void mono::callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = msg->header.frame_id;
    }
    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = tp_1.seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    //track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, msg->header.stamp);
    }
}

stereo::stereo(const std::shared_ptr<stella_vslam::system>& slam,
               const std::string& mask_img_path,
               const std::shared_ptr<stella_vslam::util::stereo_rectifier>& rectifier)
    : system(slam, mask_img_path),
      rectifier_(rectifier),
      left_sf_(node_, "camera/left/image_raw"),
      right_sf_(node_, "camera/right/image_raw") {
    use_exact_time_ = false;
    use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
    if (use_exact_time_) {
        exact_time_sync_ = std::make_shared<ExactTimeSyncPolicy::Sync>(2, left_sf_, right_sf_);
        exact_time_sync_->registerCallback(&stereo::callback, this);
    }
    else {
        approx_time_sync_ = std::make_shared<ApproximateTimeSyncPolicy::Sync>(10, left_sf_, right_sf_);
        approx_time_sync_->registerCallback(&stereo::callback, this);
    }
}

void stereo::callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = left->header.frame_id;
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
    auto cam_pose_wc = slam_->feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    //track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, left->header.stamp);
    }
}

rgbd::rgbd(const std::shared_ptr<stella_vslam::system>& slam, const std::string& mask_img_path)
    : system(slam, mask_img_path),
      color_sf_(node_, "camera/color/image_raw"),
      depth_sf_(node_, "camera/depth/image_raw") {
    use_exact_time_ = false;
    use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
    if (use_exact_time_) {
        exact_time_sync_ = std::make_shared<ExactTimeSyncPolicy::Sync>(2, color_sf_, depth_sf_);
        exact_time_sync_->registerCallback(&rgbd::callback, this);
    }
    else {
        approx_time_sync_ = std::make_shared<ApproximateTimeSyncPolicy::Sync>(10, color_sf_, depth_sf_);
        approx_time_sync_->registerCallback(&rgbd::callback, this);
    }
}

void rgbd::callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = color->header.frame_id;
    }
    auto colorcv = cv_bridge::toCvShare(color)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }
    if (depthcv.type() == CV_32FC1) {
        cv::patchNaNs(depthcv);
    }

    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = tp_1.seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track time in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, color->header.stamp);
    }
}

} // namespace stella_vslam_ros
