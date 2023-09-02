#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/yaml.h>
#include <stella_vslam_ros.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <spdlog/spdlog.h>

#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem;

namespace stella_vslam_ros {

class System : public rclcpp::Node {
public:
    System(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    System(
        const std::string& name_space,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    std::shared_ptr<stella_vslam_ros::system> slam_ros_;
    std::shared_ptr<stella_vslam::system> slam_;
    std::shared_ptr<stella_vslam::config> cfg_;
};

System::System(
    const rclcpp::NodeOptions& options)
    : System("", options) {}

System::System(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
    : Node("run_slam", name_space, options) {
    std::string vocab_file_path = declare_parameter("vocab_file_path", "");
    std::string setting_file_path = declare_parameter("setting_file_path", "");
    std::string log_level = declare_parameter("log_level", "info");
    std::string map_db_path_in = declare_parameter("map_db_path_in", "");
    std::string map_db_path_out = declare_parameter("map_db_path_out", "");
    bool disable_mapping = declare_parameter("disable_mapping", false);
    bool temporal_mapping = declare_parameter("temporal_mapping", false);

    if (vocab_file_path.empty() || setting_file_path.empty()) {
        RCLCPP_FATAL(get_logger(), "Invalid parameter");
        return;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level));

    // load configuration
    YAML::Node cfg = YAML::LoadFile(setting_file_path);
    return;
    try {
        cfg_ = std::make_shared<stella_vslam::config>(setting_file_path);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), e.what());
        return;
    }

    slam_ = std::make_shared<stella_vslam::system>(cfg_, vocab_file_path);
    bool need_initialize = true;
    if (!map_db_path_in.empty()) {
        need_initialize = false;
        const auto path = fs::path(map_db_path_in);
        if (path.extension() == ".yaml") {
            YAML::Node node = YAML::LoadFile(path);
            for (const auto& map_path : node["maps"].as<std::vector<std::string>>()) {
                slam_->load_map_database(path.parent_path() / map_path);
            }
        }
        else {
            // load the prebuilt map
            slam_->load_map_database(path);
        }
    }
    slam_->startup(need_initialize);
    if (disable_mapping) {
        slam_->disable_mapping_module();
    }
    else if (temporal_mapping) {
        slam_->enable_temporal_mapping();
        slam_->disable_loop_detector();
    }

    if (slam_->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        slam_ros_ = std::make_shared<stella_vslam_ros::mono>(slam_, "");
    }
    else {
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid setup type: " << slam_->get_camera()->get_setup_type_string());
        return;
    }
}

} // namespace stella_vslam_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(stella_vslam_ros::System)
