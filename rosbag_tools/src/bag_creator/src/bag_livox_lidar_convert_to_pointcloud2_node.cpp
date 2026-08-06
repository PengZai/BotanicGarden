#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include <livox_ros_driver/CustomMsg.h>

#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

sensor_msgs::PointCloud2 convertLivoxToPointCloud2(
    const livox_ros_driver::CustomMsg& livox_msg,
    const std::string& frame_id_override) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.points.reserve(livox_msg.point_num);

    for (const auto& point : livox_msg.points) {
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = static_cast<float>(point.reflectivity);
        cloud.points.push_back(pcl_point);
    }

    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header = livox_msg.header;
    if (!frame_id_override.empty()) {
        cloud_msg.header.frame_id = frame_id_override;
    }

    return cloud_msg;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: rosrun bag_creator bag_livox_lidar_convert_to_pointcloud2_node config.yaml" << std::endl;
        return 1;
    }

    const YAML::Node config = YAML::LoadFile(argv[1]);
    const std::string bag_path = config["bag_path"].as<std::string>();
    const std::string output_folder = config["output_folder_path"].as<std::string>();
    const std::string source_topic = config["source_topic"].as<std::string>();
    const std::string target_topic = config["target_topic"].as<std::string>();
    const std::string frame_id = config["frame_id"] ? config["frame_id"].as<std::string>() : "";
    const bool keep_source_topic = config["keep_source_topic"] ? config["keep_source_topic"].as<bool>() : false;

    const fs::path input_bag_path(bag_path);
    const fs::path output_dir(output_folder);
    const fs::path output_bag_path = output_dir / input_bag_path.filename();

    if (!fs::exists(input_bag_path)) {
        std::cerr << "Input bag does not exist: " << input_bag_path.string() << std::endl;
        return 1;
    }

    if (!fs::exists(output_dir)) {
        std::cerr << "Output folder does not exist: " << output_dir.string() << std::endl;
        return 1;
    }

    rosbag::Bag inbag;
    rosbag::Bag outbag;
    inbag.open(input_bag_path.string(), rosbag::bagmode::Read);
    outbag.open(output_bag_path.string(), rosbag::bagmode::Write);

    std::size_t converted_count = 0;
    std::size_t copied_count = 0;
    std::size_t skipped_count = 0;

    for (const rosbag::MessageInstance& message : rosbag::View(inbag)) {
        const bool is_source_topic =
            (message.getTopic() == source_topic) || ("/" + message.getTopic() == source_topic);

        if (!is_source_topic) {
            outbag.write(message.getTopic(), message.getTime(), message);
            ++copied_count;
            continue;
        }

        const livox_ros_driver::CustomMsg::ConstPtr livox_msg =
            message.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg == nullptr) {
            std::cerr << "Skipping message on " << source_topic
                      << " because it is not livox_ros_driver/CustomMsg at time "
                      << message.getTime() << std::endl;
            ++skipped_count;
            continue;
        }

        if (keep_source_topic) {
            outbag.write(message.getTopic(), message.getTime(), message);
            ++copied_count;
        }

        sensor_msgs::PointCloud2 cloud_msg = convertLivoxToPointCloud2(*livox_msg, frame_id);
        outbag.write(target_topic, livox_msg->header.stamp, cloud_msg);
        ++converted_count;
    }

    inbag.close();
    outbag.close();

    std::cout << "Wrote converted Livox clouds to: " << output_bag_path.string() << std::endl;
    std::cout << "Copied messages: " << copied_count << std::endl;
    std::cout << "Converted Livox messages: " << converted_count << std::endl;
    std::cout << "Skipped source messages: " << skipped_count << std::endl;
    return 0;
}
