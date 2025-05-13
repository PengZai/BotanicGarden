#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

using namespace std;
namespace fs = boost::filesystem;

ros::Time filenameToRosTime(const std::string& name) {
    std::string ts_str = name;
    // 去除后缀部分，比如 .tiff
    size_t dot_pos = ts_str.find('.');
    if (dot_pos != std::string::npos) {
        ts_str = ts_str.substr(0, dot_pos);
    }

    try {
        uint64_t timestamp_ns = std::stoull(ts_str);  // 纳秒
        uint32_t secs = timestamp_ns / 1e9;
        uint32_t nsecs = timestamp_ns % static_cast<uint64_t>(1e9);
        return ros::Time(secs, nsecs);
    } catch (...) {
        return ros::Time(0);
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: rosrun your_package append_depth_to_bag config.yaml" << std::endl;
        return 1;
    }

    YAML::Node config = YAML::LoadFile(argv[1]);
    std::string depth_folder = config["depth_folder_path"].as<std::string>();
    std::string bag_path = config["bag_path"].as<std::string>();
    std::string output_folder = config["output_folder_path"].as<std::string>();
    std::string depth_topic = config["depth_topic"].as<std::string>();

    fs::path input_bag(bag_path);
    fs::path output_bag = fs::path(output_folder) / input_bag.filename().replace_extension("learned_depth.bag");

    rosbag::Bag inbag, outbag;
    inbag.open(bag_path, rosbag::bagmode::Read);
    outbag.open(output_bag.string(), rosbag::bagmode::Write);

    // 1. 复制原始消息
    for (rosbag::MessageInstance const m : rosbag::View(inbag)) {
        outbag.write(m.getTopic(), m.getTime(), m);
    }

    // 2. 遍历深度图像
    cv_bridge::CvImage bridge;
    bridge.encoding = "32FC1";

    for (auto& entry : fs::directory_iterator(depth_folder)) {
        if (entry.path().extension() != ".tiff")
            continue;

        std::string filename = entry.path().stem().string();
        ros::Time stamp = filenameToRosTime(filename);
        if (stamp == ros::Time(0)) {
            std::cerr << "Skipping invalid file: " << filename << std::endl;
            continue;
        }

        cv::Mat depth = cv::imread(entry.path().string(), cv::IMREAD_UNCHANGED);
        if (depth.empty() || depth.type() != CV_32FC1) {
            std::cerr << "Invalid image: " << entry.path().string() << std::endl;
            continue;
        }

        bridge.header.stamp = stamp;
        bridge.header.frame_id = "camera_depth_optical_frame";
        bridge.image = depth;

        sensor_msgs::ImagePtr msg = bridge.toImageMsg();
        outbag.write(depth_topic, stamp, msg);
    }

    std::cout << "Finished writing to: " << output_bag.string() << std::endl;

    inbag.close();
    outbag.close();
    return 0;
}
