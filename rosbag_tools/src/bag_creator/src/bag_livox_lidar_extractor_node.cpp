#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <livox_ros_driver/CustomMsg.h>
#include <boost/foreach.hpp>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <sstream>
#include <string>
#include <vector>

struct PoseSample {
    double timestamp = 0.0;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
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

double livoxPointTimeSec(const livox_ros_driver::CustomMsg& livox_msg,
                         const uint32_t offset_time_ns)
{
    if (livox_msg.timebase != 0) {
        return static_cast<double>(livox_msg.timebase + offset_time_ns) * 1e-9;
    }
    return livox_msg.header.stamp.toSec() + static_cast<double>(offset_time_ns) * 1e-9;
}

void fillOriginalCloud(const livox_ros_driver::CustomMsg& livox_msg,
                       pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    cloud.clear();
    cloud.reserve(livox_msg.points.size());
    for (const auto& pt : livox_msg.points) {
        pcl::PointXYZI p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = static_cast<float>(pt.reflectivity);
        cloud.points.push_back(p);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
}

bool filterLivoxCloudBySOR(pcl::PointCloud<pcl::PointXYZI>& cloud,
                           const int nb_neighbors,
                           const double std_ratio)
{
    if (static_cast<int>(cloud.size()) <= nb_neighbors) {
        std::cerr << "Skipping Livox SOR: point count (" << cloud.size()
                  << ") must be greater than SOR_nb_neighbors (" << nb_neighbors << ").\n";
        return false;
    }

    pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud.makeShared());
    sor.setMeanK(nb_neighbors);
    sor.setStddevMulThresh(std_ratio);
    sor.filter(filtered_cloud);

    cloud.swap(filtered_cloud);
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    return true;
}

void filterLivoxCloudByDistance(pcl::PointCloud<pcl::PointXYZI>& cloud,
                                const double min_distance,
                                const double max_distance)
{
    if (min_distance < 0.0 && max_distance < 0.0) {
        return;
    }

    const double min_distance_sq = min_distance >= 0.0 ? min_distance * min_distance : -1.0;
    const double max_distance_sq = max_distance >= 0.0 ? max_distance * max_distance : -1.0;

    pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
    filtered_cloud.reserve(cloud.size());
    for (const pcl::PointXYZI& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        const double distance_sq =
            static_cast<double>(point.x) * static_cast<double>(point.x) +
            static_cast<double>(point.y) * static_cast<double>(point.y) +
            static_cast<double>(point.z) * static_cast<double>(point.z);
        if (min_distance_sq >= 0.0 && distance_sq < min_distance_sq) {
            continue;
        }
        if (max_distance_sq >= 0.0 && distance_sq > max_distance_sq) {
            continue;
        }

        filtered_cloud.push_back(point);
    }

    cloud.swap(filtered_cloud);
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
}

bool deskewLivoxCloud(const livox_ros_driver::CustomMsg& livox_msg,
                      const std::vector<PoseSample>& trajectory,
                      const Eigen::Isometry3d& t_pose_lidar,
                      const bool log_time_range,
                      pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    if (livox_msg.points.empty()) {
        fillOriginalCloud(livox_msg, cloud);
        return true;
    }

    const double ref_time = livox_msg.header.stamp.toSec();
    double min_point_time = std::numeric_limits<double>::infinity();
    double max_point_time = -std::numeric_limits<double>::infinity();
    for (const auto& pt : livox_msg.points) {
        const double point_time = livoxPointTimeSec(livox_msg, pt.offset_time);
        min_point_time = std::min(min_point_time, point_time);
        max_point_time = std::max(max_point_time, point_time);
    }

    if (log_time_range) {
        std::cout << "First Livox cloud point time range: header "
                  << livox_msg.header.stamp.toNSec()
                  << " + [" << (min_point_time - ref_time)
                  << ", " << (max_point_time - ref_time) << "] seconds\n";
    }

    Eigen::Isometry3d t_world_pose_ref;
    Eigen::Isometry3d t_world_pose_min;
    Eigen::Isometry3d t_world_pose_max;
    if (!interpolatePose(trajectory, ref_time, t_world_pose_ref) ||
        !interpolatePose(trajectory, min_point_time, t_world_pose_min) ||
        !interpolatePose(trajectory, max_point_time, t_world_pose_max)) {
        std::cerr << "Skipping Livox deskew at " << livox_msg.header.stamp.toNSec()
                  << ": point time range [" << min_point_time << ", " << max_point_time
                  << "] is outside trajectory coverage ["
                  << trajectory.front().timestamp << ", " << trajectory.back().timestamp
                  << "].\n";
        return false;
    }

    const Eigen::Isometry3d t_world_lidar_ref = t_world_pose_ref * t_pose_lidar;
    const Eigen::Isometry3d t_lidar_ref_world = t_world_lidar_ref.inverse();

    cloud.clear();
    cloud.reserve(livox_msg.points.size());
    for (const auto& pt : livox_msg.points) {
        pcl::PointXYZI p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = static_cast<float>(pt.reflectivity);

        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            Eigen::Isometry3d t_world_pose_point;
            const double point_time = livoxPointTimeSec(livox_msg, pt.offset_time);
            if (!interpolatePose(trajectory, point_time, t_world_pose_point)) {
                std::cerr << "Skipping Livox deskew at " << livox_msg.header.stamp.toNSec()
                          << ": point timestamp " << point_time
                          << " is outside trajectory coverage.\n";
                return false;
            }

            const Eigen::Isometry3d t_world_lidar_point = t_world_pose_point * t_pose_lidar;
            const Eigen::Vector3d point_raw(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z));
            const Eigen::Vector3d point_ref =
                t_lidar_ref_world * (t_world_lidar_point * point_raw);

            p.x = static_cast<float>(point_ref.x());
            p.y = static_cast<float>(point_ref.y());
            p.z = static_cast<float>(point_ref.z());
        }

        cloud.points.push_back(p);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    return true;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Usage: rosrun bag_creator bag_livox_lidar_extractor_node config.yaml" << std::endl;
        return 1;
    }

    rosbag::Bag bag;
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

            livox_ros_driver::CustomMsg::ConstPtr livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
            if (livox_msg != NULL) {
                pcl::PointCloud<pcl::PointXYZI> cloud;
                if (is_deskew) {
                    const bool should_log_time_range = !logged_first_cloud_time_range;
                    if (deskewLivoxCloud(*livox_msg, trajectory, t_pose_lidar,
                                         should_log_time_range, cloud)) {
                        logged_first_cloud_time_range = true;
                    } else {
                        std::cerr << "Saving original Livox cloud without deskew: "
                                  << livox_msg->header.stamp.toNSec() << std::endl;
                        fillOriginalCloud(*livox_msg, cloud);
                    }
                } else {
                    fillOriginalCloud(*livox_msg, cloud);
                }
                filterLivoxCloudByDistance(cloud,
                                           point_cloud_min_distance,
                                           point_cloud_max_distance);
                if (is_filter_point_cloud_by_sor &&
                    !filterLivoxCloudBySOR(cloud, sor_nb_neighbors, sor_std_ratio)) {
                    std::cerr << "Saving Livox cloud without SOR filtering: "
                              << livox_msg->header.stamp.toNSec() << std::endl;
                }

                std::string pcd_filename = output_folder_path +
                    std::to_string(livox_msg->header.stamp.toNSec()) + ".pcd";

                if (pcl::io::savePCDFileBinary(pcd_filename, cloud) == -1) {
                    std::cerr << "Failed to save " << pcd_filename << std::endl;
                } else {
                    std::cout << "Saved: " << pcd_filename << std::endl;
                }
            }
        }
    }

    bag.close();
    return 0;
}
