#ifdef HAVE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
#include "iridescence_viewer/viewer.h"
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
#include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem;

#ifdef USE_STACK_TRACE_LOGGER
#include <backward.hpp>
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
              const std::string& eval_log_dir,
              const std::string& map_db_path,
              const std::string& bag_path,
              const std::string& camera_topic,
              const std::string& left_topic,
              const std::string& right_topic,
              const std::string& color_topic,
              const std::string& depth_topic,
              const std::string& bag_storage_id,
              const double start_offset,
              const bool no_sleep,
              const std::string& viewer_string) {
    auto& SLAM = slam_ros->slam_;

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef HAVE_PANGOLIN_VIEWER
    std::shared_ptr<pangolin_viewer::viewer> viewer;
    if (viewer_string == "pangolin_viewer") {
        viewer = std::make_shared<pangolin_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"),
            SLAM,
            SLAM->get_frame_publisher(),
            SLAM->get_map_publisher());
    }
#endif
#ifdef HAVE_IRIDESCENCE_VIEWER
    std::shared_ptr<iridescence_viewer::viewer> iridescence_viewer;
    if (viewer_string == "iridescence_viewer") {
        iridescence_viewer = std::make_shared<iridescence_viewer::viewer>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "IridescenceViewer"),
            SLAM->get_frame_publisher(),
            SLAM->get_map_publisher());
        iridescence_viewer->add_button("Reset", [&SLAM] {
            SLAM->request_reset();
        });
        iridescence_viewer->add_button("Save and exit", [&iridescence_viewer] {
            iridescence_viewer->request_terminate();
        });
        iridescence_viewer->add_close_callback([] {
            rclcpp::shutdown();
        });
    }
#endif
#ifdef HAVE_SOCKET_PUBLISHER
    std::shared_ptr<socket_publisher::publisher> publisher;
    if (viewer_string == "socket_publisher") {
        publisher = std::make_shared<socket_publisher::publisher>(
            stella_vslam::util::yaml_optional_ref(cfg->yaml_node_, "SocketPublisher"),
            SLAM,
            SLAM->get_frame_publisher(),
            SLAM->get_map_publisher());
    }
#endif

    std::shared_ptr<std::thread> viewer_thread;
    if (viewer_string != "none") {
        // TODO: Pangolin needs to run in the main thread on OSX
        // run the viewer in another thread
        viewer_thread = std::make_shared<std::thread>([&]() {
            if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
                viewer->run();
#endif
            }
            if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
                iridescence_viewer->run();
#endif
            }
            if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
                publisher->run();
#endif
            }
            if (SLAM->terminate_is_requested()) {
                // wait until the loop BA is finished
                while (SLAM->loop_BA_is_running()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(5000));
                }
                rclcpp::shutdown();
            }
        });
    }

    // read rosbag and run SLAM
    RCLCPP_INFO_STREAM(slam_ros->node_->get_logger(), "Open " << bag_path << "(storage_id=" << bag_storage_id << ")");
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = bag_storage_id;
    rosbag2_cpp::Reader reader;
    reader.open(storage_options);
    auto metadata = reader.get_metadata();
    auto starting_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             metadata.starting_time.time_since_epoch())
                             .count()
                         + start_offset * 1e9;
    reader.seek(starting_time);
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    std::queue<std::shared_ptr<sensor_msgs::msg::Image>> que;
    double track_time;
    if (slam_ros->slam_->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        auto mono = std::static_pointer_cast<stella_vslam_ros::mono>(slam_ros);
        while (rclcpp::ok()) {
            while (reader.has_next() && que.size() < 2) {
                auto bag_message = reader.read_next();
                if (bag_message->topic_name == camera_topic) {
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
    else if (slam_ros->slam_->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        auto stereo = std::static_pointer_cast<stella_vslam_ros::stereo>(slam_ros);
        std::queue<std::shared_ptr<sensor_msgs::msg::Image>> que_right;
        while (rclcpp::ok()) {
            while (reader.has_next() && que.size() < 2) {
                auto bag_message = reader.read_next();
                if (bag_message->topic_name == left_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que.push(msg);
                }
                if (bag_message->topic_name == right_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que_right.push(msg);
                }
            }
            if (que.size() == 0) {
                break;
            }
            const auto tp_1 = std::chrono::steady_clock::now();
            stereo->left_sf_.cb(que.front());
            if (que_right.size() > 0) {
                stereo->right_sf_.cb(que_right.front());
                que_right.pop();
            }
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
    else if (slam_ros->slam_->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::RGBD) {
        auto rgbd = std::static_pointer_cast<stella_vslam_ros::rgbd>(slam_ros);
        std::queue<std::shared_ptr<sensor_msgs::msg::Image>> que_depth;
        while (rclcpp::ok()) {
            while (reader.has_next() && que.size() < 2) {
                auto bag_message = reader.read_next();
                if (bag_message->topic_name == color_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que.push(msg);
                }
                if (bag_message->topic_name == depth_topic) {
                    auto msg = std::make_shared<sensor_msgs::msg::Image>();
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, msg.get());
                    que_depth.push(msg);
                }
            }
            if (que.size() == 0) {
                break;
            }
            const auto tp_1 = std::chrono::steady_clock::now();
            rgbd->color_sf_.cb(que.front());
            if (que_depth.size() > 0) {
                rgbd->depth_sf_.cb(que_depth.front());
                que_depth.pop();
            }
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
    if (viewer_string == "pangolin_viewer") {
#ifdef HAVE_PANGOLIN_VIEWER
        viewer->request_terminate();
#endif
    }
    if (viewer_string == "iridescence_viewer") {
#ifdef HAVE_IRIDESCENCE_VIEWER
        iridescence_viewer->request_terminate();
#endif
    }
    if (viewer_string == "socket_publisher") {
#ifdef HAVE_SOCKET_PUBLISHER
        publisher->request_terminate();
#endif
    }
    if (viewer_string != "none") {
        viewer_thread->join();
    }

    // shutdown the SLAM process
    SLAM->shutdown();

    auto& track_times = slam_ros->track_times_;
    if (!eval_log_dir.empty()) {
        // output the trajectories for evaluation
        SLAM->save_frame_trajectory(eval_log_dir + "/frame_trajectory.txt", "TUM");
        SLAM->save_keyframe_trajectory(eval_log_dir + "/keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs(eval_log_dir + "/track_times.txt", std::ios::out);
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
    backward::SignalHandling sh;
#endif
    rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto bag_path = op.add<popl::Value<std::string>>("b", "bag", "rosbag2 file path");
    auto camera_topic = op.add<popl::Value<std::string>>("", "camera", "image topic name for monoculari", "camera/image_raw");
    auto left_topic = op.add<popl::Value<std::string>>("", "left", "left image topic name for stereo", "camera/left/image_raw");
    auto right_topic = op.add<popl::Value<std::string>>("", "right", "right image topic name for stereo", "camera/right/image_raw");
    auto color_topic = op.add<popl::Value<std::string>>("", "color", "color image topic name for RGBD", "camera/color/image_raw");
    auto depth_topic = op.add<popl::Value<std::string>>("", "depth", "depth image topic name for RGBD", "camera/depth/image_raw");
    auto bag_storage_id = op.add<popl::Value<std::string>>("", "storage-id", "rosbag2 storage id (default: sqlite3)", "sqlite3");
    auto start_offset = op.add<popl::Value<double>>("", "start-offset", "start offset", 0.0);
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto log_level = op.add<popl::Value<std::string>>("", "log-level", "log level", "info");
    auto eval_log_dir = op.add<popl::Value<std::string>>("", "eval-log-dir", "store trajectory and tracking times at this path (Specify the directory where it exists.)", "");
    auto map_db_path_in = op.add<popl::Value<std::string>>("i", "map-db-in", "load a map from this path", "");
    auto map_db_path_out = op.add<popl::Value<std::string>>("o", "map-db-out", "store a map database at this path after slam", "");
    auto disable_mapping = op.add<popl::Switch>("", "disable-mapping", "disable mapping");
    auto temporal_mapping = op.add<popl::Switch>("", "temporal-mapping", "enable temporal mapping");
    auto rectify = op.add<popl::Switch>("r", "rectify", "rectify stereo image");
    auto viewer = op.add<popl::Value<std::string>>("", "viewer", "viewer [iridescence_viewer, pangolin_viewer, socket_publisher, none]");
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

    // viewer
    std::string viewer_string;
    if (viewer->is_set()) {
        viewer_string = viewer->value();
        if (viewer_string != "pangolin_viewer"
            && viewer_string != "socket_publisher"
            && viewer_string != "iridescence_viewer"
            && viewer_string != "none") {
            std::cerr << "invalid arguments (--viewer)" << std::endl
                      << std::endl
                      << op << std::endl;
            return EXIT_FAILURE;
        }
#ifndef HAVE_PANGOLIN_VIEWER
        if (viewer_string == "pangolin_viewer") {
            std::cerr << "pangolin_viewer not linked" << std::endl
                      << std::endl
                      << op << std::endl;
            return EXIT_FAILURE;
        }
#endif
#ifndef HAVE_IRIDESCENCE_VIEWER
        if (viewer_string == "iridescence_viewer") {
            std::cerr << "iridescence_viewer not linked" << std::endl
                      << std::endl
                      << op << std::endl;
            return EXIT_FAILURE;
        }
#endif
#ifndef HAVE_SOCKET_PUBLISHER
        if (viewer_string == "socket_publisher") {
            std::cerr << "socket_publisher not linked" << std::endl
                      << std::endl
                      << op << std::endl;
            return EXIT_FAILURE;
        }
#endif
    }
    else {
#ifdef HAVE_IRIDESCENCE_VIEWER
        viewer_string = "iridescence_viewer";
#elif defined(HAVE_PANGOLIN_VIEWER)
        viewer_string = "pangolin_viewer";
#elif defined(HAVE_SOCKET_PUBLISHER)
        viewer_string = "socket_publisher";
#endif
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level->value()));

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
    else if (temporal_mapping->is_set()) {
        slam->enable_temporal_mapping();
        slam->disable_loop_detector();
    }

    auto node = std::make_shared<rclcpp::Node>("run_slam");
    std::shared_ptr<stella_vslam_ros::system> slam_ros;
    if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        slam_ros = std::make_shared<stella_vslam_ros::mono>(slam, node.get(), mask_img_path->value());
    }
    else if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Stereo) {
        auto rectifier = rectify->value() ? std::make_shared<stella_vslam::util::stereo_rectifier>(cfg, slam->get_camera()) : nullptr;
        slam_ros = std::make_shared<stella_vslam_ros::stereo>(slam, node.get(), mask_img_path->value(), rectifier);
    }
    else if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::RGBD) {
        slam_ros = std::make_shared<stella_vslam_ros::rgbd>(slam, node.get(), mask_img_path->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + slam->get_camera()->get_setup_type_string());
    }

    // run tracking
    tracking(
        slam_ros,
        cfg, eval_log_dir->value(),
        map_db_path_out->value(),
        bag_path->value(),
        camera_topic->value(),
        left_topic->value(),
        right_topic->value(),
        color_topic->value(),
        depth_topic->value(),
        bag_storage_id->value(),
        start_offset->value(),
        no_sleep->is_set(),
        viewer_string);

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
