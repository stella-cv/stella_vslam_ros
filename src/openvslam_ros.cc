#include <openvslam_ros.h>
#include <openvslam/publish/map_publisher.h>

#include <chrono>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>

namespace openvslam_ros {
system::system(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : SLAM_(cfg, vocab_file_path), cfg_(cfg), private_nh_("~"), it_(nh_), tp_0_(std::chrono::steady_clock::now()),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)),
      pose_pub_(private_nh_.advertise<nav_msgs::Odometry>("camera_pose", 1)),
      map_to_odom_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>()),
      tf_(std::make_unique<tf2_ros::Buffer>()),
      transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_)) {
    init_pose_sub_ = nh_.subscribe(
        "/initialpose", 1, &system::init_pose_callback, this);
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const ros::Time& stamp) {
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
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_link_;
    pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine);
    pose_pub_.publish(pose_msg);

    // Send map->odom transform. Set publish_tf to false if not using TF
    if (publish_tf_) {
        try {
            auto camera_to_odom = tf_->lookupTransform(camera_link_, odom_frame_, stamp, ros::Duration(0.0));
            Eigen::Affine3d camera_to_odom_affine = tf2::transformToEigen(camera_to_odom.transform);

            auto map_to_odom_msg = tf2::eigenToTransform(map_to_camera_affine * camera_to_odom_affine);
            auto transform_timestamp = stamp + ros::Duration(transform_tolerance_);
            map_to_odom_msg.header.stamp = transform_timestamp;
            map_to_odom_msg.header.frame_id = map_frame_;
            map_to_odom_msg.child_frame_id = odom_frame_;
            map_to_odom_broadcaster_->sendTransform(map_to_odom_msg);
        }
        catch (tf2::TransformException& ex) {
            ROS_ERROR("Transform failed: %s", ex.what());
        }
    }
}

void system::setParams() {
    odom_frame_ = std::string("odom");
    private_nh_.param("odom_frame", odom_frame_, odom_frame_);

    map_frame_ = std::string("map");
    private_nh_.param("map_frame", map_frame_, map_frame_);

    base_link_ = std::string("base_footprint");
    private_nh_.param("base_link", base_link_, base_link_);

    // Set publish_tf to false if not using TF
    publish_tf_ = true;
    private_nh_.param("publish_tf_", publish_tf_, publish_tf_);

    // Publish pose's timestamp in the future
    transform_tolerance_ = 0.5;
    private_nh_.param("transform_tolerance", transform_tolerance_, transform_tolerance_);
}

void system::init_pose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (camera_link_.empty()) {
        ROS_ERROR("Camera link is not set: no images were received yet");
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
            map_frame_, msg->header.frame_id, msg->header.stamp,
            ros::Duration(0.0));
        map_to_initialpose_frame_affine = tf2::transformToEigen(
            map_to_initialpose_frame.transform);
    }
    catch (tf2::TransformException& ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
        return;
    }

    Eigen::Affine3d base_link_to_camera_affine;
    try {
        auto base_link_to_camera = tf_->lookupTransform(
            base_link_, camera_link_, msg->header.stamp,
            ros::Duration(0.0));
        base_link_to_camera_affine = tf2::transformToEigen(base_link_to_camera.transform);
    }
    catch (tf2::TransformException& ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
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

    if (!SLAM_.update_pose(cam_pose_cv)) {
        ROS_ERROR("Can not set initial pose");
    }
}

mono::mono(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
    sub_ = it_.subscribe("camera/image_raw", 1, &mono::callback, this);
}
void mono::callback(const sensor_msgs::ImageConstPtr& msg) {
    if (camera_link_.empty()) {
        camera_link_ = msg->header.frame_id;
    }
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, msg->header.stamp);
    }
}

stereo::stereo(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path,
               const bool rectify)
    : system(cfg, vocab_file_path, mask_img_path),
      rectifier_(rectify ? std::make_shared<openvslam::util::stereo_rectifier>(cfg) : nullptr),
      left_sf_(it_, "camera/left/image_raw", 1),
      right_sf_(it_, "camera/right/image_raw", 1),
      sync_(SyncPolicy(10), left_sf_, right_sf_) {
    sync_.registerCallback(&stereo::callback, this);
}

void stereo::callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right) {
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

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, left->header.stamp);
    }
}

rgbd::rgbd(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path),
      color_sf_(it_, "camera/color/image_raw", 1),
      depth_sf_(it_, "camera/depth/image_raw", 1),
      sync_(SyncPolicy(10), color_sf_, depth_sf_) {
    sync_.registerCallback(&rgbd::callback, this);
}

void rgbd::callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth) {
    if (camera_link_.empty()) {
        camera_link_ = color->header.frame_id;
    }
    auto colorcv = cv_bridge::toCvShare(color)->image;
    auto depthcv = cv_bridge::toCvShare(depth)->image;
    if (colorcv.empty() || depthcv.empty()) {
        return;
    }
    if (depthcv.type() == CV_32FC1) {
        cv::patchNaNs(depthcv);
    }

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = SLAM_.feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, color->header.stamp);
    }
}
} // namespace openvslam_ros
