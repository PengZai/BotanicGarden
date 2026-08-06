#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

ros::Time filenameToRosTime(const std::string& name) {
    std::string ts_str = name;
    const std::size_t dot_pos = ts_str.find('.');
    if (dot_pos != std::string::npos) {
        ts_str = ts_str.substr(0, dot_pos);
    }

    try {
        const uint64_t raw_timestamp = std::stoull(ts_str);
        const std::size_t digits = ts_str.size();

        uint64_t timestamp_ns = raw_timestamp;
        if (digits <= 10) {
            return ros::Time(static_cast<uint32_t>(raw_timestamp), 0);
        }
        if (digits <= 13) {
            timestamp_ns = raw_timestamp * 1000000ULL;
        } else if (digits <= 16) {
            timestamp_ns = raw_timestamp * 1000ULL;
        }

        const uint32_t secs = static_cast<uint32_t>(timestamp_ns / 1000000000ULL);
        const uint32_t nsecs = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
        return ros::Time(secs, nsecs);
    } catch (...) {
        return ros::Time(0);
    }
}

void copyBagMessages(const std::string& input_bag_path, rosbag::Bag& outbag) {
    rosbag::Bag inbag;
    inbag.open(input_bag_path, rosbag::bagmode::Read);

    for (const rosbag::MessageInstance& m : rosbag::View(inbag)) {
        outbag.write(m.getTopic(), m.getTime(), m);
    }

    inbag.close();
}

void normalizeFieldNames(sensor_msgs::PointCloud2& cloud_msg) {
    static const std::map<std::string, std::string> field_aliases = {
        {"i", "intensity"},
        {"intensity", "intensity"},
        {"r", "ring"},
        {"ring", "ring"},
        {"t", "time"},
        {"timestamp", "time"},
        {"time", "time"}
    };

    for (sensor_msgs::PointField& field : cloud_msg.fields) {
        const auto it = field_aliases.find(field.name);
        if (it != field_aliases.end()) {
            field.name = it->second;
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: rosrun bag_creator bag_velodyne_lidar_append_node config.yaml" << std::endl;
        return 1;
    }

    YAML::Node config = YAML::LoadFile(argv[1]);
    const std::string lidar_folder = config["lidar_folder_path"].as<std::string>();
    const std::string bag_path = config["bag_path"].as<std::string>();
    const std::string output_folder = config["output_folder_path"].as<std::string>();
    const std::string lidar_topic = config["lidar_topic"].as<std::string>();
    const std::string frame_id = config["frame_id"] ? config["frame_id"].as<std::string>() : "";

    const fs::path input_bag_path(bag_path);
    const fs::path output_dir(output_folder);
    const fs::path output_bag_path = output_dir / input_bag_path.filename();

    if (!fs::exists(output_dir)) {
        std::cerr << "Output folder does not exist: " << output_dir.string() << std::endl;
        return 1;
    }

    if (!fs::exists(lidar_folder) || !fs::is_directory(lidar_folder)) {
        std::cerr << "Lidar folder does not exist: " << lidar_folder << std::endl;
        return 1;
    }

    const bool input_bag_exists = fs::exists(input_bag_path);
    const bool target_bag_exists = fs::exists(output_bag_path);

    rosbag::Bag outbag;
    if (target_bag_exists) {
        outbag.open(output_bag_path.string(), rosbag::bagmode::Append);
        std::cout << "Appending lidar data to existing bag: " << output_bag_path.string() << std::endl;
    } else {
        outbag.open(output_bag_path.string(), rosbag::bagmode::Write);
        if (input_bag_exists) {
            std::cout << "Copying existing bag to: " << output_bag_path.string() << std::endl;
            copyBagMessages(input_bag_path.string(), outbag);
        } else {
            std::cout << "Source bag not found. Creating new bag: " << output_bag_path.string() << std::endl;
        }
    }

    std::vector<fs::path> pcd_files;
    for (const auto& entry : fs::directory_iterator(lidar_folder)) {
        if (entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path());
        }
    }

    std::sort(pcd_files.begin(), pcd_files.end());

    std::size_t written_count = 0;
    std::size_t total_size = pcd_files.size();
    for (const fs::path& pcd_path : pcd_files) {
        const std::string filename = pcd_path.stem().string();
        const ros::Time stamp = filenameToRosTime(filename);
        if (stamp == ros::Time(0)) {
            std::cerr << "Skipping invalid file: " << pcd_path.string() << std::endl;
            continue;
        }

        pcl::PCLPointCloud2 pcl_cloud;
        if (pcl::io::loadPCDFile(pcd_path.string(), pcl_cloud) == -1) {
            std::cerr << "Failed to load PCD file: " << pcd_path.string() << std::endl;
            continue;
        }

        sensor_msgs::PointCloud2 cloud_msg;
        pcl_conversions::fromPCL(pcl_cloud, cloud_msg);
        normalizeFieldNames(cloud_msg);
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = frame_id;

        outbag.write(lidar_topic, stamp, cloud_msg);
        ++written_count;
        std::cout << "Processed " << pcd_path.string() << " (" << written_count << "/" << total_size << ")" << std::endl;
    }

    outbag.close();
    std::cout << "Finished writing " << written_count << " lidar messages to: "
              << output_bag_path.string() << std::endl;
    return 0;
}
