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
    // 去除后缀部分，比如 .png
    size_t dot_pos = ts_str.find('.');
    if (dot_pos != std::string::npos) {
        ts_str = ts_str.substr(0, dot_pos);
    }

    try {
        uint64_t timestamp_ns = std::stoull(ts_str);  // 纳秒
        uint32_t secs = timestamp_ns / 1000000000ULL;
        uint32_t nsecs = timestamp_ns % 1000000000ULL;
        return ros::Time(secs, nsecs);
    } catch (...) {
        return ros::Time(0);
    }
}

int main(int argc, char** argv) {

    std::cout << "test" << std::endl;

    if (argc != 2) {
        std::cerr << "Usage: rosrun your_package append_rgb_to_bag config.yaml" << std::endl;
        return 1;
    }
    std::cout << "test" << std::endl;
    YAML::Node config = YAML::LoadFile(argv[1]);
    std::cout << "test" << std::endl;
    std::string rgb_folder = config["rgb_folder_path"].as<std::string>();
    std::string bag_path = config["bag_path"].as<std::string>();
    std::string output_folder = config["output_folder_path"].as<std::string>();
    std::string rgb_topic = config["rgb_topic"].as<std::string>();

    fs::path input_bag(bag_path);
    fs::path output_dir(output_folder);
    fs::path output_bag = fs::path(output_folder) / input_bag.filename();

    rosbag::Bag inbag, outbag;
    inbag.open(bag_path, rosbag::bagmode::Read);
    outbag.open(output_bag.string(), rosbag::bagmode::Write);

    // 1. 复制原始消息
    for (rosbag::MessageInstance const m : rosbag::View(inbag)) {
        outbag.write(m.getTopic(), m.getTime(), m);
    }

    // 2. 遍历 RGB 图像
    for (auto& entry : fs::directory_iterator(rgb_folder)) {
        std::string ext = entry.path().extension().string();
        if (!(ext == ".png" || ext == ".jpg" || ext == ".jpeg"))
            continue;

        std::string filename = entry.path().stem().string();
        std::cout << "Processing " << filename << std::endl;

        ros::Time stamp = filenameToRosTime(filename);
        if (stamp == ros::Time(0)) {
            std::cerr << "Skipping invalid file: " << filename << std::endl;
            continue;
        }

        cv::Mat rgb = cv::imread(entry.path().string(), cv::IMREAD_COLOR);
        if (rgb.empty() || rgb.type() != CV_8UC3) {
            std::cerr << "Invalid image: " << entry.path().string() << std::endl;
            continue;
        }

        cv_bridge::CvImage bridge;
        bridge.encoding = "bgr8";  // 对应 OpenCV 的 CV_8UC3
        bridge.header.stamp = stamp;
        bridge.header.frame_id = "camera_rgb_optical_frame";
        bridge.image = rgb;

        sensor_msgs::ImagePtr msg = bridge.toImageMsg();
        outbag.write(rgb_topic, stamp, msg);
    }

    std::cout << "Finished writing to: " << output_bag.string() << std::endl;

    inbag.close();
    outbag.close();
    return 0;
}
