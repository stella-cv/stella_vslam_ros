#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/util/yaml.h>
#include <stella_vslam_ros.h>

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>
#include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem;

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

void tracking(const std::shared_ptr<stella_vslam_ros::system>& slam_ros,
              const std::shared_ptr<stella_vslam::config>& cfg,
              const bool eval_log,
              const std::string& map_db_path,
              const std::string& bag_path,
              const std::string& camera_name,
              const std::string& bag_storage_id,
              const bool no_sleep) {
    auto& SLAM = slam_ros->slam_;

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"), SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"), SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
#endif

    // TODO: Pangolin needs to run in the main thread on OSX
    // run the viewer in another thread
#ifdef USE_PANGOLIN_VIEWER
    std::thread thread([&]() {
        viewer.run();
        if (SLAM->terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM->loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            rclcpp::shutdown();
        }
    });
#elif USE_SOCKET_PUBLISHER
    std::thread thread([&]() {
        publisher.run();
        if (SLAM->terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM->loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            rclcpp::shutdown();
        }
    });
#endif

    // read rosbag and run SLAM
    if (slam_ros->slam_->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        auto mono = std::static_pointer_cast<stella_vslam_ros::mono>(slam_ros);
        RCLCPP_INFO_STREAM(slam_ros->node_->get_logger(), "Open " << bag_path << "(storage_id=" << bag_storage_id << ")");
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = bag_storage_id;
        rosbag2_cpp::Reader reader;
        reader.open(storage_options);
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
        std::queue<std::shared_ptr<sensor_msgs::msg::Image>> que;
        double track_time;

        while (rclcpp::ok()) {
            while (reader.has_next() && que.size() < 2) {
                auto bag_message = reader.read_next();
                if (bag_message->topic_name == "/" + camera_name + "/image_raw") {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que.push(msg);
                }
            }
            if (que.size() == 0) {
                break;
            }
            const auto tp_1 = std::chrono::steady_clock::now();
            mono->callback(que.front());
            const auto tp_2 = std::chrono::steady_clock::now();
            track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

            // wait until the timestamp of the next frame
            if (!no_sleep && que.size() == 2) {
                const auto wait_time = (rclcpp::Time(que.back()->header.stamp) - rclcpp::Time(que.front()->header.stamp)).seconds() + track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }
            que.pop();
        }
    }
    else {
        throw std::runtime_error("Invalid setup type: " + slam_ros->slam_->get_camera()->get_setup_type_string());
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
    SLAM->shutdown();

    auto& track_times = slam_ros->track_times_;
    if (eval_log) {
        // output the trajectories for evaluation
        SLAM->save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM->save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM->save_map_database(map_db_path);
    }

    if (track_times.size()) {
        std::sort(track_times.begin(), track_times.end());
        const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
        RCLCPP_DEBUG(slam_ros->node_->get_logger(), "Median tracking time: %f [s] ", track_times.at(track_times.size() / 2));
        RCLCPP_DEBUG(slam_ros->node_->get_logger(), "Mean tracking time: %f [s] ", total_track_time / track_times.size());
    }
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif
    rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto bag_path = op.add<popl::Value<std::string>>("b", "bag", "rosbag2 file path");
    auto camera_name = op.add<popl::Value<std::string>>("", "camera", "image topic name must be <camera_name>/image_raw", "camera");
    auto bag_storage_id = op.add<popl::Value<std::string>>("", "storage-id", "rosbag2 storage id (default: sqlite3)", "sqlite3");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path_in = op.add<popl::Value<std::string>>("i", "map-db-in", "load a map from this path", "");
    auto map_db_path_out = op.add<popl::Value<std::string>>("o", "map-db-out", "store a map database at this path after slam", "");
    auto disable_mapping = op.add<popl::Switch>("", "disable-mapping", "disable mapping");
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
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
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
    std::shared_ptr<stella_vslam::config> cfg;
    try {
        cfg = std::make_shared<stella_vslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_file_path->value());
    bool need_initialize = true;
    if (map_db_path_in->is_set()) {
        need_initialize = false;
        const auto path = fs::path(map_db_path_in->value());
        if (path.extension() == ".yaml") {
            YAML::Node node = YAML::LoadFile(path);
            for (const auto& map_path : node["maps"].as<std::vector<std::string>>()) {
                slam->load_map_database(path.parent_path() / map_path);
            }
        }
        else {
            // load the prebuilt map
            slam->load_map_database(path);
        }
    }
    slam->startup(need_initialize);
    if (disable_mapping->is_set()) {
        slam->disable_mapping_module();
    }

    std::shared_ptr<stella_vslam_ros::system> slam_ros;
    if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        slam_ros = std::make_shared<stella_vslam_ros::mono>(slam, mask_img_path->value());
    }
    else if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        auto rectifier = rectify->value() ? std::make_shared<stella_vslam::util::stereo_rectifier>(cfg, slam->get_camera()) : nullptr;
        slam_ros = std::make_shared<stella_vslam_ros::stereo>(slam, mask_img_path->value(), rectifier);
    }
    else if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::RGBD) {
        slam_ros = std::make_shared<stella_vslam_ros::rgbd>(slam, mask_img_path->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + slam->get_camera()->get_setup_type_string());
    }

    // run tracking
    tracking(
        slam_ros,
        cfg, eval_log->is_set(),
        map_db_path_out->value(),
        bag_path->value(),
        camera_name->value(),
        bag_storage_id->value(),
        no_sleep->is_set());

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
