#include <stella_vslam_ros.h>
#include <stella_vslam/publish/map_publisher.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/data/keyframe.h>

#include <chrono>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Geometry>

namespace stella_vslam_ros {
system::system(const std::shared_ptr<stella_vslam::system>& slam,
               const std::string& mask_img_path)
    : slam_(slam), private_nh_("~"), it_(nh_), tp_0_(std::chrono::steady_clock::now()),
      mask_(mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)),
      pose_pub_(private_nh_.advertise<nav_msgs::Odometry>("camera_pose", 1)),
      pc_pub_(private_nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", 1)),
      keyframes_pub_(private_nh_.advertise<geometry_msgs::PoseArray>("keyframes", 1)),
      keyframes_2d_pub_(private_nh_.advertise<geometry_msgs::PoseArray>("keyframes_2d", 1)),
      map_to_odom_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>()),
      tf_(std::make_unique<tf2_ros::Buffer>()),
      transform_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_)) {
    init_pose_sub_ = nh_.subscribe(
        "/initialpose", 1, &system::init_pose_callback, this);
    setParams();
    rot_ros_to_cv_map_frame_ = (Eigen::Matrix3d() << 0, 0, 1,
                                -1, 0, 0,
                                0, -1, 0)
                                   .finished();
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const ros::Time& stamp) {
    // Extract rotation matrix and translation vector from
    Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
    Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
    Eigen::Affine3d map_to_camera_affine(trans * rot);

    // Transform map frame from CV coordinate system to ROS coordinate system
    map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame_);

    // Create odometry message and update it with current camera pose
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.child_frame_id = camera_frame_;
    pose_msg.pose.pose = tf2::toMsg(map_to_camera_affine * rot_ros_to_cv_map_frame_.inverse());
    pose_pub_.publish(pose_msg);

    // Send map->odom transform. Set publish_tf to false if not using TF
    if (publish_tf_) {
        try {
            auto camera_to_odom = tf_->lookupTransform(camera_optical_frame_, odom_frame_, stamp, ros::Duration(0.0));
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

void system::publish_pointcloud(const ros::Time& stamp) {
    std::vector<std::shared_ptr<stella_vslam::data::landmark>> landmarks;
    std::set<std::shared_ptr<stella_vslam::data::landmark>> local_landmarks;
    slam_->get_map_publisher()->get_landmarks(landmarks, local_landmarks);
    pcl::PointCloud<pcl::PointXYZ> points;
    for (const auto lm : landmarks) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        const Eigen::Vector3d pos_w = rot_ros_to_cv_map_frame_ * lm->get_pos_in_world();
        points.push_back(pcl::PointXYZ(pos_w.x(), pos_w.y(), pos_w.z()));
    }
    sensor_msgs::PointCloud2 pcout;
    pcl::toROSMsg(points, pcout);
    pcout.header.frame_id = map_frame_;
    pcout.header.stamp = stamp;
    pc_pub_.publish(pcout);
}

void system::publish_keyframes(const ros::Time& stamp) {
    Eigen::Vector3d normal_vector;
    normal_vector << 0, 0, 1;
    geometry_msgs::PoseArray keyframes_msg;
    geometry_msgs::PoseArray keyframes_2d_msg;
    keyframes_msg.header.stamp = stamp;
    keyframes_msg.header.frame_id = map_frame_;
    keyframes_2d_msg.header = keyframes_msg.header;
    std::vector<std::shared_ptr<stella_vslam::data::keyframe>> all_keyfrms;
    slam_->get_map_publisher()->get_keyframes(all_keyfrms);
    for (const auto keyfrm : all_keyfrms) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }
        Eigen::Matrix4d cam_pose_wc = keyfrm->get_pose_wc();
        Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
        Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
        Eigen::Affine3d map_to_camera_affine(trans * rot);
        Eigen::Affine3d pose_affine = rot_ros_to_cv_map_frame_ * map_to_camera_affine * rot_ros_to_cv_map_frame_.inverse();
        keyframes_msg.poses.push_back(tf2::toMsg(pose_affine));
        pose_affine.translation() = pose_affine.translation() - pose_affine.translation().dot(normal_vector) * normal_vector;
        keyframes_2d_msg.poses.push_back(tf2::toMsg(pose_affine));
    }
    keyframes_pub_.publish(keyframes_msg);
    keyframes_2d_pub_.publish(keyframes_2d_msg);
}

void system::setParams() {
    odom_frame_ = std::string("odom");
    private_nh_.param("odom_frame", odom_frame_, odom_frame_);

    map_frame_ = std::string("map");
    private_nh_.param("map_frame", map_frame_, map_frame_);

    base_link_ = std::string("base_footprint");
    private_nh_.param("base_link", base_link_, base_link_);

    camera_frame_ = std::string("camera_frame");
    private_nh_.param("camera_frame", camera_frame_, camera_frame_);

    // Set publish_tf to false if not using TF
    publish_tf_ = true;
    private_nh_.param("publish_tf", publish_tf_, publish_tf_);

    // Set publish_pointcloud_ to true if publish pointcloud
    publish_pointcloud_ = false;
    private_nh_.param("publish_pointcloud", publish_pointcloud_, publish_pointcloud_);

    publish_keyframes_ = true;
    private_nh_.param("publish_keyframes", publish_keyframes_, publish_keyframes_);

    // Publish pose's timestamp in the future
    transform_tolerance_ = 0.5;
    private_nh_.param("transform_tolerance", transform_tolerance_, transform_tolerance_);
}

void system::init_pose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (camera_optical_frame_.empty()) {
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
            base_link_, camera_optical_frame_, msg->header.stamp,
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
    Eigen::Matrix4d cam_pose_cv = (rot_ros_to_cv_map_frame_.inverse() * map_to_initialpose_frame_affine
                                   * initialpose_affine * base_link_to_camera_affine)
                                      .matrix();

    const Eigen::Vector3d normal_vector = (Eigen::Vector3d() << 0., 1., 0.).finished();
    if (!slam_->relocalize_by_pose_2d(cam_pose_cv, normal_vector)) {
        ROS_ERROR("Can not set initial pose");
    }
}

mono::mono(const std::shared_ptr<stella_vslam::system>& slam,
           const std::string& mask_img_path)
    : system(slam, mask_img_path) {
    sub_ = it_.subscribe("camera/image_raw", 1, &mono::callback, this);
}
void mono::callback(const sensor_msgs::ImageConstPtr& msg) {
    if (camera_optical_frame_.empty()) {
        camera_optical_frame_ = msg->header.frame_id;
    }
    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, msg->header.stamp);
    }

    if (publish_pointcloud_) {
        publish_pointcloud(msg->header.stamp);
    }
    if (publish_keyframes_) {
        publish_keyframes(msg->header.stamp);
    }
}

stereo::stereo(const std::shared_ptr<stella_vslam::system>& slam,
               const std::string& mask_img_path,
               const std::shared_ptr<stella_vslam::util::stereo_rectifier>& rectifier)
    : system(slam, mask_img_path),
      rectifier_(rectifier),
      left_sf_(it_, "camera/left/image_raw", 1),
      right_sf_(it_, "camera/right/image_raw", 1) {
    use_exact_time_ = false;
    private_nh_.param("use_exact_time", use_exact_time_, use_exact_time_);
    if (use_exact_time_) {
        exact_time_sync_ = std::make_shared<ExactTimeSyncPolicy::Sync>(2, left_sf_, right_sf_);
        exact_time_sync_->registerCallback(&stereo::callback, this);
    }
    else {
        approx_time_sync_ = std::make_shared<ApproximateTimeSyncPolicy::Sync>(10, left_sf_, right_sf_);
        approx_time_sync_->registerCallback(&stereo::callback, this);
    }
}

void stereo::callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right) {
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

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_stereo_frame(leftcv, rightcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, left->header.stamp);
    }

    if (publish_pointcloud_) {
        publish_pointcloud(left->header.stamp);
    }
    if (publish_keyframes_) {
        publish_keyframes(left->header.stamp);
    }
}

rgbd::rgbd(const std::shared_ptr<stella_vslam::system>& slam,
           const std::string& mask_img_path)
    : system(slam, mask_img_path),
      color_sf_(it_, "camera/color/image_raw", 1),
      depth_sf_(it_, "camera/depth/image_raw", 1) {
    use_exact_time_ = false;
    private_nh_.param("use_exact_time", use_exact_time_, use_exact_time_);
    if (use_exact_time_) {
        exact_time_sync_ = std::make_shared<ExactTimeSyncPolicy::Sync>(2, color_sf_, depth_sf_);
        exact_time_sync_->registerCallback(&rgbd::callback, this);
    }
    else {
        approx_time_sync_ = std::make_shared<ApproximateTimeSyncPolicy::Sync>(10, color_sf_, depth_sf_);
        approx_time_sync_->registerCallback(&rgbd::callback, this);
    }
}

void rgbd::callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth) {
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

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0_).count();

    // input the current frame and estimate the camera pose
    auto cam_pose_wc = slam_->feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

    const auto tp_2 = std::chrono::steady_clock::now();

    const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    track_times_.push_back(track_time);

    if (cam_pose_wc) {
        publish_pose(*cam_pose_wc, color->header.stamp);
    }

    if (publish_pointcloud_) {
        publish_pointcloud(color->header.stamp);
    }
    if (publish_keyframes_) {
        publish_keyframes(color->header.stamp);
    }
}
} // namespace stella_vslam_ros
