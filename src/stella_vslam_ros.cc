#include <stella_vslam_ros.h>
#include <stella_vslam/publish/map_publisher.h>
#include <stella_vslam/data/keyframe.h>

#include <chrono>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>

namespace {
Eigen::Affine3d project_to_xy_plane(const Eigen::Affine3d& affine) {
    Eigen::Matrix4d mat = affine.matrix();
    mat(2, 3) = 0.0;
    Eigen::Translation<double, 3> trans(mat.col(3).head<3>());
    double rx = mat(0, 0);
    double ry = mat(1, 0);
    double yaw = std::atan2(ry, rx);
    return trans * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}
} // namespace

namespace stella_vslam_ros {
system::system(const std::shared_ptr<stella_vslam::system>& slam,
               rclcpp::Node* node,
               const std::string& mask_img_path)
    : slam_(slam), node_(node), custom_qos_(rmw_qos_profile_sensor_data),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)),
      pose_pub_(node_->create_publisher<nav_msgs::msg::Odometry>("~/camera_pose", 1)),
      keyframes_pub_(node_->create_publisher<geometry_msgs::msg::PoseArray>("~/keyframes", 1)),
      keyframes_2d_pub_(node_->create_publisher<geometry_msgs::msg::PoseArray>("~/keyframes_2d", 1)),
      atlas_cloud_pub_(node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/atlas", 1)),
      map_to_odom_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
      tf_(std::make_unique<tf2_ros::Buffer>(node_->get_clock())),
      transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_)) {
    custom_qos_.depth = 1;
    init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&system::init_pose_callback,
                  this, std::placeholders::_1));
    setParams();
    rot_ros_to_cv_map_frame_ = (Eigen::Matrix3d() << 0, 0, 1,
                                -1, 0, 0,
                                0, -1, 0)
                                   .finished();
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp) {
    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
    Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
    Eigen::Affine3d map_to_camera_affine(trans * rot);

    // Transform map frame from CV coordinate system to ROS coordinate system
    map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame_);

    // Create odometry message and update it with current camera pose
    nav_msgs::msg::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_frame_;
    pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine * rot_ros_to_cv_map_frame_.inverse());
    pose_pub_->publish(pose_msg);

    // Send map->odom transform. Set publish_tf to false if not using TF
    if (publish_tf_) {
        try {
            auto camera_to_odom = tf_->lookupTransform(
                camera_optical_frame_, odom_frame_, tf2_ros::fromMsg(builtin_interfaces::msg::Time(stamp)),
                tf2::durationFromSec(0.0));
            Eigen::Affine3d camera_to_odom_affine = tf2::transformToEigen(camera_to_odom.transform);

            geometry_msgs::msg::TransformStamped map_to_odom_msg;
            if (odom2d_) {
                Eigen::Affine3d map_to_camera_affine_2d = project_to_xy_plane(map_to_camera_affine * rot_ros_to_cv_map_frame_.inverse()) * rot_ros_to_cv_map_frame_;
                Eigen::Affine3d camera_to_odom_affine_2d = (project_to_xy_plane(camera_to_odom_affine.inverse() * rot_ros_to_cv_map_frame_.inverse()) * rot_ros_to_cv_map_frame_).inverse();
                Eigen::Affine3d map_to_odom_affine_2d = map_to_camera_affine_2d * camera_to_odom_affine_2d;
                map_to_odom_msg = tf2::eigenToTransform(map_to_odom_affine_2d);
            }
            else {
                map_to_odom_msg = tf2::eigenToTransform(map_to_camera_affine * camera_to_odom_affine);
            }
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

void system::publish_keyframes(const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseArray keyframes_msg;
    geometry_msgs::msg::PoseArray keyframes_2d_msg;
    keyframes_msg.header.stamp = stamp;
    keyframes_msg.header.frame_id = map_frame_;
    keyframes_2d_msg.header = keyframes_msg.header;
    std::vector<std::shared_ptr<stella_vslam::data::keyframe>> all_keyfrms;
    slam_->get_map_publisher()->get_keyframes(all_keyfrms);
    for (const auto& keyfrm : all_keyfrms) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }
        Eigen::Matrix4d cam_pose_wc = keyfrm->get_pose_wc();
        Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
        Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
        Eigen::Affine3d map_to_camera_affine(trans * rot);
        Eigen::Affine3d pose_affine = rot_ros_to_cv_map_frame_ * map_to_camera_affine * rot_ros_to_cv_map_frame_.inverse();
        keyframes_msg.poses.push_back(tf2::toMsg(pose_affine));
        keyframes_2d_msg.poses.push_back(tf2::toMsg(project_to_xy_plane(pose_affine)));
    }
    keyframes_pub_->publish(keyframes_msg);
    keyframes_2d_pub_->publish(keyframes_2d_msg);
}

void system::publish_atlas_as_cloud(const rclcpp::Time& stamp) {
    std::vector<std::shared_ptr<stella_vslam::data::landmark> > landmarks;
    std::set<std::shared_ptr<stella_vslam::data::landmark> > local_landmarks;
    slam_->get_map_publisher()->get_landmarks(landmarks, local_landmarks);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = map_frame_;
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;
    cloud_msg.width = 1;
    cloud_msg.height = landmarks.size();

    sensor_msgs::PointCloud2Modifier pcl2(cloud_msg);
    pcl2.setPointCloud2FieldsByString(1, "xyz");
    pcl2.resize(landmarks.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (size_t key = 0; key < landmarks.size(); ++key, ++iter_x, ++iter_y, ++iter_z) {
        auto lm = landmarks.at(key);
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        const auto pos = lm->get_pos_in_world();
        *iter_x = pos.x();
        *iter_y = pos.y();
        *iter_z = pos.z();
    }
    atlas_cloud_pub_->publish(cloud_msg);
}

void system::setParams() {
    odom_frame_ = std::string("odom");
    odom_frame_ = node_->declare_parameter("odom_frame", odom_frame_);

    map_frame_ = std::string("map");
    map_frame_ = node_->declare_parameter("map_frame", map_frame_);

    robot_base_frame_ = std::string("base_link");
    robot_base_frame_ = node_->declare_parameter("robot_base_frame", robot_base_frame_);

    camera_frame_ = std::string("camera_frame");
    camera_frame_ = node_->declare_parameter("camera_frame", camera_frame_);

    publish_tf_ = true;
    publish_tf_ = node_->declare_parameter("publish_tf", publish_tf_);

    odom2d_ = false;
    odom2d_ = node_->declare_parameter("odom2d", odom2d_);

    publish_keyframes_ = true;
    publish_keyframes_ = node_->declare_parameter("publish_keyframes", publish_keyframes_);

    transform_tolerance_ = 0.5;
    transform_tolerance_ = node_->declare_parameter("transform_tolerance", transform_tolerance_);

    encoding_ = "";
    encoding_ = node_->declare_parameter("encoding", encoding_);
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

    Eigen::Affine3d robot_base_frame_to_camera_affine;
    try {
        auto robot_base_frame_to_camera = tf_->lookupTransform(
            robot_base_frame_, camera_optical_frame_, tf2_ros::fromMsg(msg->header.stamp),
            tf2::durationFromSec(0.0));
        robot_base_frame_to_camera_affine = tf2::transformToEigen(robot_base_frame_to_camera.transform);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform failed: " << ex.what());
        return;
    }

    // Target transform is map_cv -> camera_link and known parameters are following:
    //   rot_cv_to_ros_map_frame: T(map_cv -> map)
    //   map_to_initialpose_frame_affine: T(map -> `msg->header.frame_id`)
    //   initialpose_affine: T(`msg->header.frame_id` -> base_link)
    //   robot_base_frame_to_camera_affine: T(base_link -> camera_link)
    // The flow of the transformation is as follows:
    //   map_cv -> map -> `msg->header.frame_id` -> base_link -> camera_link
    Eigen::Matrix4d cam_pose_cv = (rot_cv_to_ros_map_frame * map_to_initialpose_frame_affine
                                   * initialpose_affine * robot_base_frame_to_camera_affine)
                                      .matrix();

    const Eigen::Vector3d normal_vector = (Eigen::Vector3d() << 0., 1., 0.).finished();
    if (!slam_->relocalize_by_pose_2d(cam_pose_cv, normal_vector)) {
        RCLCPP_ERROR(node_->get_logger(), "Can not set initial pose");
    }
}

mono::mono(const std::shared_ptr<stella_vslam::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path)
    : system(slam, node, mask_img_path) {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_), custom_qos_);
    raw_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", qos, [this](sensor_msgs::msg::Image::UniquePtr msg_unique_ptr) { callback(std::move(msg_unique_ptr)); });
}
void mono::callback(sensor_msgs::msg::Image::UniquePtr msg_unique_ptr) {
    sensor_msgs::msg::Image::ConstSharedPtr msg = std::move(msg_unique_ptr);
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = msg->header.frame_id;
    }
    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_monocular_frame(cv_bridge::toCvShare(msg, encoding_)->image, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, msg->header.stamp);
    }
    if (publish_keyframes_) {
        publish_keyframes(msg->header.stamp);
    }
    if (node_->count_subscribers(atlas_cloud_pub_->get_topic_name()) > 0) {
        publish_atlas_as_cloud(msg->header.stamp);
    }
}

void mono::callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = msg->header.frame_id;
    }
    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_monocular_frame(cv_bridge::toCvShare(msg, encoding_)->image, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, msg->header.stamp);
    }
    if (publish_keyframes_) {
        publish_keyframes(msg->header.stamp);
    }
    if (node_->count_subscribers(atlas_cloud_pub_->get_topic_name()) > 0) {
        publish_atlas_as_cloud(msg->header.stamp);
    }
}

stereo::stereo(const std::shared_ptr<stella_vslam::system>& slam,
               rclcpp::Node* node,
               const std::string& mask_img_path,
               const std::shared_ptr<stella_vslam::util::stereo_rectifier>& rectifier)
    : system(slam, node, mask_img_path),
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
    auto leftcv = cv_bridge::toCvShare(left, encoding_)->image;
    auto rightcv = cv_bridge::toCvShare(right, encoding_)->image;
    if (leftcv.empty() || rightcv.empty()) {
        return;
    }

    if (rectifier_) {
        rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
    }

    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = rclcpp::Time(left->header.stamp).seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track times in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, left->header.stamp);
    }
    if (publish_keyframes_) {
        publish_keyframes(left->header.stamp);
    }
    if (node_->count_subscribers(atlas_cloud_pub_->get_topic_name()) > 0) {
        publish_atlas_as_cloud(left->header.stamp);
    }
}

rgbd::rgbd(const std::shared_ptr<stella_vslam::system>& slam,
           rclcpp::Node* node,
           const std::string& mask_img_path)
    : system(slam, node, mask_img_path),
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
    auto colorcv = cv_bridge::toCvShare(color, encoding_)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }
    if (depthcv.type() == CV_32FC1) {
        cv::patchNaNs(depthcv);
    }

    const rclcpp::Time tp_1 = node_->now();
    const double timestamp = rclcpp::Time(color->header.stamp).seconds();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const rclcpp::Time tp_2 = node_->now();
    const double track_time = (tp_2 - tp_1).seconds();

    // track time in seconds
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, color->header.stamp);
    }
    if (publish_keyframes_) {
        publish_keyframes(color->header.stamp);
    }
    if (node_->count_subscribers(atlas_cloud_pub_->get_topic_name()) > 0) {
        publish_atlas_as_cloud(color->header.stamp);
    }
}

} // namespace stella_vslam_ros
