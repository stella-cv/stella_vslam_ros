#ifdef HAVE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#endif
#ifdef HAVE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

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
    virtual ~System();

    std::shared_ptr<stella_vslam_ros::system> slam_ros_;
    std::shared_ptr<stella_vslam::system> slam_;
    std::shared_ptr<stella_vslam::config> cfg_;
    std::string map_db_path_out_;
    std::string viewer_string_;
    std::shared_ptr<std::thread> viewer_thread_;
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> viewer_;
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    std::shared_ptr<socket_publisher::publisher> publisher_;
#endif
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
    map_db_path_out_ = declare_parameter("map_db_path_out", "");
    bool disable_mapping = declare_parameter("disable_mapping", false);
    bool temporal_mapping = declare_parameter("temporal_mapping", false);
    std::string viewer = declare_parameter("viewer", "none");

    if (vocab_file_path.empty() || setting_file_path.empty()) {
        RCLCPP_FATAL(get_logger(), "Invalid parameter");
        return;
    }

    // viewer
    if (!viewer.empty()) {
        viewer_string_ = viewer;
        if (viewer_string_ != "pangolin_viewer" && viewer_string_ != "socket_publisher" && viewer_string_ != "none") {
            RCLCPP_FATAL(get_logger(), "invalid arguments (--viewer)");
            return;
        }
#ifndef HAVE_PANGOLIN_VIEWER
        if (viewer_string_ == "pangolin_viewer") {
            RCLCPP_FATAL(get_logger(), "pangolin_viewer not linked");
            return;
        }
#endif
#ifndef HAVE_SOCKET_PUBLISHER
        if (viewer_string_ == "socket_publisher") {
            RCLCPP_FATAL(get_logger(), "socket_publisher not linked");
            return;
        }
#endif
    }
    else {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer_string_ = "pangolin_viewer";
#elif defined(HAVE_SOCKET_PUBLISHER)
        viewer_string_ = "socket_publisher";
#endif
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level));

    // load configuration
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
        slam_ros_ = std::make_shared<stella_vslam_ros::mono>(slam_, this, "");
    }
    else {
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid setup type: " << slam_->get_camera()->get_setup_type_string());
        return;
    }

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef HAVE_PANGOLIN_VIEWER
    if (viewer_string_ == "pangolin_viewer") {
        viewer_ = std::make_shared<pangolin_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg_->yaml_node_, "PangolinViewer"),
            slam_,
            slam_->get_frame_publisher(),
            slam_->get_map_publisher());
    }
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    if (viewer_string_ == "socket_publisher") {
        publisher_ = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg_->yaml_node_, "SocketPublisher"),
            slam_,
            slam_->get_frame_publisher(),
            slam_->get_map_publisher());
    }
#endif

    if (viewer_string_ != "none") {
        // TODO: Pangolin needs to run in the main thread on OSX
        // run the viewer in another thread
        viewer_thread_ = std::make_shared<std::thread>([&, this]() {
            if (viewer_string_ == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
                viewer_->run();
#endif
            }
            if (viewer_string_ == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
                publisher_->run();
#endif
            }
            if (slam_->terminate_is_requested()) {
                // wait until the loop BA is finished
                while (slam_->loop_BA_is_running()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(5000));
                }
                rclcpp::shutdown();
            }
        });
    }
}

System::~System() {
    // automatically close the viewer
    if (viewer_string_ == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer_->request_terminate();
#endif
    }
    if (viewer_string_ == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        publisher_->request_terminate();
#endif
    }
    if (viewer_string_ != "none") {
        viewer_thread_->join();
    }

    // shutdown the SLAM process
    slam_->shutdown();

    if (!map_db_path_out_.empty()) {
        // output the map database
        slam_->save_map_database(map_db_path_out_);
    }
}

} // namespace stella_vslam_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(stella_vslam_ros::System)
