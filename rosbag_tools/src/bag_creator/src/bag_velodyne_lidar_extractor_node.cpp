#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <boost/foreach.hpp>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>
#include <string>
#include <vector>

struct PoseSample {
    double timestamp = 0.0;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

struct CloudFieldOffsets {
    int x = -1;
    int y = -1;
    int z = -1;
    int time = -1;
};

Sophus::SE3d poseSampleToSE3(const PoseSample& sample)
{
    return Sophus::SE3d(sample.rotation, sample.translation);
}

Eigen::Isometry3d se3ToIsometry(const Sophus::SE3d& se3)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = se3.rotationMatrix();
    pose.translation() = se3.translation();
    return pose;
}

bool isAbsolutePath(const std::string& path)
{
    return !path.empty() && path[0] == '/';
}

std::string joinPath(const std::string& folder, const std::string& name)
{
    if (folder.empty() || isAbsolutePath(name)) {
        return name;
    }
    if (folder.back() == '/') {
        return folder + name;
    }
    return folder + "/" + name;
}

bool parseTransform(const YAML::Node& node, Eigen::Isometry3d& transform)
{
    if (!node || !node.IsSequence() || node.size() != 4) {
        return false;
    }

    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    for (std::size_t r = 0; r < 4; ++r) {
        if (!node[r].IsSequence() || node[r].size() != 4) {
            return false;
        }
        for (std::size_t c = 0; c < 4; ++c) {
            matrix(static_cast<int>(r), static_cast<int>(c)) = node[r][c].as<double>();
        }
    }

    transform = Eigen::Isometry3d(matrix);
    return true;
}

std::vector<PoseSample> loadTrajectory(const std::string& trajectory_path)
{
    std::ifstream file(trajectory_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open trajectory file: " + trajectory_path);
    }

    std::vector<PoseSample> trajectory;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        PoseSample sample;
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
        if (!(ss >> sample.timestamp
                 >> sample.translation.x()
                 >> sample.translation.y()
                 >> sample.translation.z()
                 >> qx >> qy >> qz >> qw)) {
            continue;
        }

        sample.rotation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
        trajectory.push_back(sample);
    }

    std::sort(trajectory.begin(), trajectory.end(),
              [](const PoseSample& lhs, const PoseSample& rhs) {
                  return lhs.timestamp < rhs.timestamp;
              });

    return trajectory;
}

bool interpolatePose(const std::vector<PoseSample>& trajectory,
                     const double timestamp,
                     Eigen::Isometry3d& pose)
{
    if (trajectory.empty() ||
        timestamp < trajectory.front().timestamp ||
        timestamp > trajectory.back().timestamp) {
        return false;
    }

    const auto upper = std::lower_bound(
        trajectory.begin(), trajectory.end(), timestamp,
        [](const PoseSample& sample, const double value) {
            return sample.timestamp < value;
        });

    if (upper == trajectory.begin()) {
        pose = Eigen::Isometry3d::Identity();
        pose.linear() = trajectory.front().rotation.toRotationMatrix();
        pose.translation() = trajectory.front().translation;
        return true;
    }

    if (upper == trajectory.end()) {
        pose = Eigen::Isometry3d::Identity();
        pose.linear() = trajectory.back().rotation.toRotationMatrix();
        pose.translation() = trajectory.back().translation;
        return true;
    }

    const PoseSample& prev = *(upper - 1);
    const PoseSample& next = *upper;
    const double span = next.timestamp - prev.timestamp;
    const double alpha = span > 0.0 ? (timestamp - prev.timestamp) / span : 0.0;

    const Sophus::SE3d t0 = poseSampleToSE3(prev);
    const Sophus::SE3d t1 = poseSampleToSE3(next);
    const Sophus::SE3d interpolated =
        t0 * Sophus::SE3d::exp(alpha * (t0.inverse() * t1).log());

    pose = se3ToIsometry(interpolated);
    return true;
}

CloudFieldOffsets getFieldOffsets(const sensor_msgs::PointCloud2& cloud_msg)
{
    CloudFieldOffsets offsets;
    for (const sensor_msgs::PointField& field : cloud_msg.fields) {
        if (field.datatype != sensor_msgs::PointField::FLOAT32 || field.count != 1) {
            continue;
        }

        if (field.name == "x") {
            offsets.x = static_cast<int>(field.offset);
        } else if (field.name == "y") {
            offsets.y = static_cast<int>(field.offset);
        } else if (field.name == "z") {
            offsets.z = static_cast<int>(field.offset);
        } else if (field.name == "time") {
            offsets.time = static_cast<int>(field.offset);
        }
    }
    return offsets;
}

float readFloat32(const uint8_t* data)
{
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(float));
    return value;
}

void writeFloat32(uint8_t* data, const float value)
{
    std::memcpy(data, &value, sizeof(float));
}

bool savePointCloud2AsPcd(const sensor_msgs::PointCloud2& cloud_msg,
                          const std::string& filename)
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(cloud_msg, pcl_cloud);
    pcl::PCDWriter writer;
    return writer.writeBinary(filename, pcl_cloud) == 0;
}

bool filterPointCloud2BySOR(sensor_msgs::PointCloud2& cloud_msg,
                            const int nb_neighbors,
                            const double std_ratio)
{
    if (cloud_msg.is_bigendian) {
        std::cerr << "Skipping SOR for big-endian PointCloud2; only little-endian clouds are supported.\n";
        return false;
    }

    const CloudFieldOffsets offsets = getFieldOffsets(cloud_msg);
    if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0) {
        std::cerr << "Skipping SOR at " << cloud_msg.header.stamp.toNSec()
                  << ": expected float32 x/y/z fields.\n";
        return false;
    }

    const std::size_t point_count =
        static_cast<std::size_t>(cloud_msg.width) * static_cast<std::size_t>(cloud_msg.height);
    if (point_count == 0) {
        return true;
    }

    const std::size_t required_size =
        (point_count - 1) * static_cast<std::size_t>(cloud_msg.point_step) + cloud_msg.point_step;
    if (cloud_msg.data.size() < required_size) {
        std::cerr << "Skipping SOR for malformed cloud at " << cloud_msg.header.stamp.toNSec()
                  << ": data buffer is smaller than width*height*point_step.\n";
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::size_t> original_indices;
    xyz_cloud->reserve(point_count);
    original_indices.reserve(point_count);

    for (std::size_t i = 0; i < point_count; ++i) {
        const uint8_t* point_data = cloud_msg.data.data() + i * cloud_msg.point_step;
        const float x = readFloat32(point_data + offsets.x);
        const float y = readFloat32(point_data + offsets.y);
        const float z = readFloat32(point_data + offsets.z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        xyz_cloud->push_back(pcl::PointXYZ(x, y, z));
        original_indices.push_back(i);
    }

    if (static_cast<int>(xyz_cloud->size()) <= nb_neighbors) {
        std::cerr << "Skipping SOR at " << cloud_msg.header.stamp.toNSec()
                  << ": finite point count (" << xyz_cloud->size()
                  << ") must be greater than SOR_nb_neighbors (" << nb_neighbors << ").\n";
        return false;
    }

    std::vector<int> kept_indices;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(xyz_cloud);
    sor.setMeanK(nb_neighbors);
    sor.setStddevMulThresh(std_ratio);
    sor.filter(kept_indices);

    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(static_cast<std::size_t>(kept_indices.size()) * cloud_msg.point_step);
    for (const int kept_index : kept_indices) {
        const std::size_t original_index = original_indices.at(static_cast<std::size_t>(kept_index));
        const uint8_t* point_begin = cloud_msg.data.data() + original_index * cloud_msg.point_step;
        filtered_data.insert(filtered_data.end(), point_begin, point_begin + cloud_msg.point_step);
    }

    cloud_msg.data.swap(filtered_data);
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(kept_indices.size());
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.is_dense = true;
    return true;
}

bool filterPointCloud2ByDistance(sensor_msgs::PointCloud2& cloud_msg,
                                 const double min_distance,
                                 const double max_distance)
{
    if (min_distance < 0.0 && max_distance < 0.0) {
        return true;
    }

    if (cloud_msg.is_bigendian) {
        std::cerr << "Skipping distance filter for big-endian PointCloud2; only little-endian clouds are supported.\n";
        return false;
    }

    const CloudFieldOffsets offsets = getFieldOffsets(cloud_msg);
    if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0) {
        std::cerr << "Skipping distance filter at " << cloud_msg.header.stamp.toNSec()
                  << ": expected float32 x/y/z fields.\n";
        return false;
    }

    const std::size_t point_count =
        static_cast<std::size_t>(cloud_msg.width) * static_cast<std::size_t>(cloud_msg.height);
    if (point_count == 0) {
        return true;
    }

    const std::size_t required_size =
        (point_count - 1) * static_cast<std::size_t>(cloud_msg.point_step) + cloud_msg.point_step;
    if (cloud_msg.data.size() < required_size) {
        std::cerr << "Skipping distance filter for malformed cloud at " << cloud_msg.header.stamp.toNSec()
                  << ": data buffer is smaller than width*height*point_step.\n";
        return false;
    }

    const double min_distance_sq = min_distance >= 0.0 ? min_distance * min_distance : -1.0;
    const double max_distance_sq = max_distance >= 0.0 ? max_distance * max_distance : -1.0;

    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(cloud_msg.data.size());
    for (std::size_t i = 0; i < point_count; ++i) {
        const uint8_t* point_begin = cloud_msg.data.data() + i * cloud_msg.point_step;
        const float x = readFloat32(point_begin + offsets.x);
        const float y = readFloat32(point_begin + offsets.y);
        const float z = readFloat32(point_begin + offsets.z);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        const double distance_sq =
            static_cast<double>(x) * static_cast<double>(x) +
            static_cast<double>(y) * static_cast<double>(y) +
            static_cast<double>(z) * static_cast<double>(z);
        if (min_distance_sq >= 0.0 && distance_sq < min_distance_sq) {
            continue;
        }
        if (max_distance_sq >= 0.0 && distance_sq > max_distance_sq) {
            continue;
        }

        filtered_data.insert(filtered_data.end(), point_begin, point_begin + cloud_msg.point_step);
    }

    cloud_msg.data.swap(filtered_data);
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(cloud_msg.data.size() / cloud_msg.point_step);
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.is_dense = true;
    return true;
}

bool deskewCloud(sensor_msgs::PointCloud2& cloud_msg,
                 const std::vector<PoseSample>& trajectory,
                 const Eigen::Isometry3d& t_pose_lidar,
                 const bool log_time_range)
{
    if (cloud_msg.is_bigendian) {
        std::cerr << "Skipping big-endian PointCloud2; only little-endian clouds are supported.\n";
        return false;
    }

    const CloudFieldOffsets offsets = getFieldOffsets(cloud_msg);
    if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0 || offsets.time < 0) {
        std::cerr << "Skipping cloud at " << cloud_msg.header.stamp.toNSec()
                  << ": expected float32 x/y/z/time fields.\n";
        return false;
    }

    const std::size_t point_count =
        static_cast<std::size_t>(cloud_msg.width) * static_cast<std::size_t>(cloud_msg.height);
    if (point_count == 0) {
        return true;
    }

    const std::size_t required_size =
        (point_count - 1) * static_cast<std::size_t>(cloud_msg.point_step) + cloud_msg.point_step;
    if (cloud_msg.data.size() < required_size) {
        std::cerr << "Skipping malformed cloud at " << cloud_msg.header.stamp.toNSec()
                  << ": data buffer is smaller than width*height*point_step.\n";
        return false;
    }

    double min_offset = std::numeric_limits<double>::infinity();
    double max_offset = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < point_count; ++i) {
        const uint8_t* point_data = cloud_msg.data.data() + i * cloud_msg.point_step;
        const double point_offset = static_cast<double>(readFloat32(point_data + offsets.time));
        if (std::isfinite(point_offset)) {
            min_offset = std::min(min_offset, point_offset);
            max_offset = std::max(max_offset, point_offset);
        }
    }

    if (!std::isfinite(min_offset) || !std::isfinite(max_offset)) {
        std::cerr << "Skipping cloud at " << cloud_msg.header.stamp.toNSec()
                  << ": no finite per-point time values.\n";
        return false;
    }

    const double ref_time = cloud_msg.header.stamp.toSec();
    const double min_point_time = ref_time + min_offset;
    const double max_point_time = ref_time + max_offset;

    if (log_time_range) {
        std::cout << "First cloud point time range: header "
                  << cloud_msg.header.stamp.toNSec()
                  << " + [" << min_offset << ", " << max_offset << "] seconds\n";
    }

    Eigen::Isometry3d t_world_pose_ref;
    Eigen::Isometry3d t_world_pose_min;
    Eigen::Isometry3d t_world_pose_max;
    if (!interpolatePose(trajectory, ref_time, t_world_pose_ref) ||
        !interpolatePose(trajectory, min_point_time, t_world_pose_min) ||
        !interpolatePose(trajectory, max_point_time, t_world_pose_max)) {
        std::cerr << "Skipping cloud at " << cloud_msg.header.stamp.toNSec()
                  << ": point time range [" << min_point_time << ", " << max_point_time
                  << "] is outside trajectory coverage ["
                  << trajectory.front().timestamp << ", " << trajectory.back().timestamp
                  << "].\n";
        return false;
    }

    const Eigen::Isometry3d t_world_lidar_ref = t_world_pose_ref * t_pose_lidar;
    const Eigen::Isometry3d t_lidar_ref_world = t_world_lidar_ref.inverse();

    for (std::size_t i = 0; i < point_count; ++i) {
        uint8_t* point_data = cloud_msg.data.data() + i * cloud_msg.point_step;
        const float x = readFloat32(point_data + offsets.x);
        const float y = readFloat32(point_data + offsets.y);
        const float z = readFloat32(point_data + offsets.z);
        const float time_offset = readFloat32(point_data + offsets.time);

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            !std::isfinite(time_offset)) {
            continue;
        }

        Eigen::Isometry3d t_world_pose_point;
        const double point_time = ref_time + static_cast<double>(time_offset);
        if (!interpolatePose(trajectory, point_time, t_world_pose_point)) {
            std::cerr << "Skipping cloud at " << cloud_msg.header.stamp.toNSec()
                      << ": point timestamp " << point_time
                      << " is outside trajectory coverage.\n";
            return false;
        }

        const Eigen::Isometry3d t_world_lidar_point = t_world_pose_point * t_pose_lidar;
        const Eigen::Vector3d point_raw(static_cast<double>(x),
                                        static_cast<double>(y),
                                        static_cast<double>(z));
        const Eigen::Vector3d point_ref =
            t_lidar_ref_world * (t_world_lidar_point * point_raw);

        writeFloat32(point_data + offsets.x, static_cast<float>(point_ref.x()));
        writeFloat32(point_data + offsets.y, static_cast<float>(point_ref.y()));
        writeFloat32(point_data + offsets.z, static_cast<float>(point_ref.z()));
    }

    return true;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Usage: rosrun bag_creator bag_velodyne_lidar_extractor_node config.yaml" << std::endl;
        return 1;
    }

    std::string config_file_path = std::string(argv[1]);
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::string input_bag_path = config_node["input_bag_path"].as<std::string>();
    std::string bag_name = config_node["bag_name"].as<std::string>();
    std::string bag_path = input_bag_path + bag_name;

    std::string lidar_topic = config_node["lidar_topic"].as<std::string>();
    std::string output_folder_path = config_node["output_folder_path"].as<std::string>();
    int down_sample_ratio = config_node["down_sample_ratio"].as<int>();
    const bool is_deskew = config_node["isDeskew"] ? config_node["isDeskew"].as<bool>() : false;
    const bool is_filter_point_cloud_by_sor =
        config_node["isFilterPointCloudBySOR"] ? config_node["isFilterPointCloudBySOR"].as<bool>() : false;
    const int sor_nb_neighbors =
        config_node["SOR_nb_neighbors"] ? config_node["SOR_nb_neighbors"].as<int>() : 20;
    const double sor_std_ratio =
        config_node["SOR_std_ratio"] ? config_node["SOR_std_ratio"].as<double>() : 2.0;
    const double point_cloud_min_distance =
        config_node["point_cloud_min_distance"] ? config_node["point_cloud_min_distance"].as<double>() : -1.0;
    const double point_cloud_max_distance =
        config_node["point_cloud_max_distance"] ? config_node["point_cloud_max_distance"].as<double>() : -1.0;

    if (is_filter_point_cloud_by_sor && (sor_nb_neighbors <= 0 || sor_std_ratio <= 0.0)) {
        std::cerr << "Invalid SOR config: SOR_nb_neighbors and SOR_std_ratio must be positive.\n";
        return 1;
    }
    if ((point_cloud_min_distance < -1.0 || point_cloud_max_distance < -1.0) ||
        (point_cloud_min_distance >= 0.0 && point_cloud_max_distance >= 0.0 &&
         point_cloud_min_distance > point_cloud_max_distance)) {
        std::cerr << "Invalid distance filter config: use -1 to disable a limit, "
                  << "otherwise require 0 <= min <= max.\n";
        return 1;
    }

    std::vector<PoseSample> trajectory;
    Eigen::Isometry3d t_pose_lidar = Eigen::Isometry3d::Identity();
    if (is_deskew) {
        const std::string trajectory_path =
            joinPath(input_bag_path, config_node["trajectory_path"].as<std::string>());
        trajectory = loadTrajectory(trajectory_path);
        if (trajectory.size() < 2) {
            std::cerr << "Deskew requires at least two trajectory samples: "
                      << trajectory_path << std::endl;
            return 1;
        }
        if (!parseTransform(config_node["T_pose_lidar"], t_pose_lidar)) {
            std::cerr << "Invalid or missing T_pose_lidar in config.\n";
            return 1;
        }
        std::cout << "Loaded " << trajectory.size()
                  << " trajectory poses from " << trajectory_path << std::endl;
    }

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics = {lidar_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int count = 0;
    bool logged_first_cloud_time_range = false;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == lidar_topic || ("/" + m.getTopic() == lidar_topic)) {
            count++;
            if (count % down_sample_ratio != 0)
                continue;

            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg != nullptr) {
                sensor_msgs::PointCloud2 output_cloud = *cloud_msg;
                if (is_deskew) {
                    sensor_msgs::PointCloud2 deskewed_cloud = *cloud_msg;
                    const bool should_log_time_range = !logged_first_cloud_time_range;
                    if (deskewCloud(deskewed_cloud, trajectory, t_pose_lidar, should_log_time_range)) {
                        output_cloud = deskewed_cloud;
                        logged_first_cloud_time_range = true;
                    } else {
                        std::cerr << "Saving original cloud without deskew: "
                                  << cloud_msg->header.stamp.toNSec() << std::endl;
                    }
                }
                if (!filterPointCloud2ByDistance(output_cloud,
                                                 point_cloud_min_distance,
                                                 point_cloud_max_distance)) {
                    std::cerr << "Saving cloud without distance filtering: "
                              << cloud_msg->header.stamp.toNSec() << std::endl;
                }
                if (is_filter_point_cloud_by_sor &&
                    !filterPointCloud2BySOR(output_cloud, sor_nb_neighbors, sor_std_ratio)) {
                    std::cerr << "Saving cloud without SOR filtering: "
                              << cloud_msg->header.stamp.toNSec() << std::endl;
                }

                std::string filename = output_folder_path +
                    std::to_string(cloud_msg->header.stamp.toNSec()) + ".pcd";

                if (!savePointCloud2AsPcd(output_cloud, filename)) {
                    std::cerr << "Failed to save: " << filename << std::endl;
                    continue;
                }
                std::cout << "Saved: " << filename << std::endl;
            }
        }
    }

    bag.close();
    return 0;
}
