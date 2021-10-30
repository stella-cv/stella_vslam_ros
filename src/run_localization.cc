#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/util/yaml.h>
#include <openvslam_ros.h>

#include <iostream>
#include <chrono>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void localization(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
                  const std::string& mask_img_path, const std::string& map_db_path, const bool mapping, const bool rectify) {
    std::shared_ptr<openvslam_ros::system> ros;
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        ros = std::make_shared<openvslam_ros::mono>(cfg, vocab_file_path, mask_img_path);
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo) {
        ros = std::make_shared<openvslam_ros::stereo>(cfg, vocab_file_path, mask_img_path, rectify);
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD) {
        ros = std::make_shared<openvslam_ros::rgbd>(cfg, vocab_file_path, mask_img_path);
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

    auto& SLAM = ros->SLAM_;
    // load the prebuilt map
    SLAM.load_map_database(map_db_path);
    // startup the SLAM process (it does not need initialization of a map)
    SLAM.startup(false);
    // select to activate the mapping module or not
    if (mapping) {
        SLAM.enable_mapping_module();
    }
    else {
        SLAM.disable_mapping_module();
    }

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(openvslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(openvslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    // TODO: Pangolin needs to run in the main thread on OSX
    // run the viewer in this thread
#ifdef USE_PANGOLIN_VIEWER
    std::thread thread([&]() {
        viewer.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            rclcpp::shutdown();
        }
    });
#elif USE_SOCKET_PUBLISHER
    std::thread thread([&]() {
        publisher.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            rclcpp::shutdown();
        }
    });
#endif

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
        ros->exec_.spin_some();
        rate.sleep();
    }

    // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
    viewer.request_terminate();
    thread.join();
#elif USE_SOCKET_PUBLISHER
    publisher.request_terminate();
    thread.join();
#endif

    // shutdown the SLAM process
    SLAM.shutdown();

    auto& track_times = ros->track_times_;
    if (track_times.size()) {
        std::sort(track_times.begin(), track_times.end());
        const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
        RCLCPP_DEBUG(ros->node_->get_logger(), "Median tracking time: %f [s] ", track_times.at(track_times.size() / 2));
        RCLCPP_DEBUG(ros->node_->get_logger(), "Mean tracking time: %f [s] ", total_track_time / track_times.size());
    }
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif
    rclcpp::init(argc, argv);
    rclcpp::uninstall_signal_handlers();

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "path to a prebuilt map database");
    auto mapping = op.add<popl::Switch>("", "mapping", "perform mapping as well as localization");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto rectify = op.add<popl::Switch>("r", "rectify", "rectify stereo image");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !setting_file_path->is_set() || !map_db_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    localization(cfg, vocab_file_path->value(), mask_img_path->value(), map_db_path->value(), mapping->is_set(), rectify->value());

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
