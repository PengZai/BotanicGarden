#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

struct PoseRecord {
    ros::Time timestamp;
    geometry_msgs::Pose pose;
    bool valid;
};

bool isFinitePose(const geometry_msgs::Pose& pose)
{
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
}

Eigen::Quaterniond quatFromPose(const geometry_msgs::Pose& pose)
{
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (q.norm() == 0.0) {
        return Eigen::Quaterniond::Identity();
    }
    return q.normalized();
}

geometry_msgs::Pose interpolatePose(const geometry_msgs::Pose& start,
                                   const geometry_msgs::Pose& end,
                                   const double alpha)
{
    const Eigen::Vector3d start_pos(start.position.x, start.position.y, start.position.z);
    const Eigen::Vector3d end_pos(end.position.x, end.position.y, end.position.z);
    const Eigen::Vector3d interp_pos = start_pos + alpha * (end_pos - start_pos);

    Eigen::Quaterniond q0 = quatFromPose(start);
    Eigen::Quaterniond q1 = quatFromPose(end);
    if (q0.dot(q1) < 0.0) {
        q1.coeffs() *= -1.0;
    }
    const Eigen::Quaterniond q_interp = q0.slerp(alpha, q1).normalized();

    geometry_msgs::Pose pose_out;
    pose_out.position.x = interp_pos.x();
    pose_out.position.y = interp_pos.y();
    pose_out.position.z = interp_pos.z();
    pose_out.orientation.w = q_interp.w();
    pose_out.orientation.x = q_interp.x();
    pose_out.orientation.y = q_interp.y();
    pose_out.orientation.z = q_interp.z();

    return pose_out;
}

bool interpolatePoseFromNeighbors(const std::vector<PoseRecord>& records,
                                const size_t index,
                                geometry_msgs::Pose& pose_out)
{
    const auto& target = records[index];
    const double target_time = target.timestamp.toSec();

    int prev_idx = -1;
    for (int i = static_cast<int>(index) - 1; i >= 0; --i) {
        if (records[i].valid) {
            prev_idx = i;
            break;
        }
    }

    int next_idx = -1;
    for (size_t i = index + 1; i < records.size(); ++i) {
        if (records[i].valid) {
            next_idx = static_cast<int>(i);
            break;
        }
    }

    if (prev_idx == -1 && next_idx == -1) {
        return false;
    }

    if (prev_idx == -1) {
        pose_out = records[next_idx].pose;
        return true;
    }

    if (next_idx == -1) {
        pose_out = records[prev_idx].pose;
        return true;
    }

    const auto& prev = records[prev_idx];
    const auto& next = records[next_idx];
    const double prev_time = prev.timestamp.toSec();
    const double next_time = next.timestamp.toSec();
    const double time_span = next_time - prev_time;

    if (std::fabs(time_span) < std::numeric_limits<double>::epsilon()) {
        pose_out = prev.pose;
        return true;
    }

    const double alpha = (target_time - prev_time) / time_span;
    pose_out = interpolatePose(prev.pose, next.pose, std::max(0.0, std::min(1.0, alpha)));
    return true;
}

void writePose(std::ofstream& csv_file,
               std::ofstream& txt_file,
               const ros::Time& timestamp,
               const geometry_msgs::Pose& pose)
{
    const double vx = 0.0, vy = 0.0, vz = 0.0;
    const double bwx = 0.0, bwy = 0.0, bwz = 0.0;
    const double bax = 0.0, bay = 0.0, baz = 0.0;

    const auto& pos = pose.position;
    const auto& ori = pose.orientation;

    csv_file << timestamp.toNSec() << ","
             << pos.x << "," << pos.y << "," << pos.z << ","
             << ori.w << "," << ori.x << "," << ori.y << "," << ori.z << ","
             << vx << "," << vy << "," << vz << ","
             << bwx << "," << bwy << "," << bwz << ","
             << bax << "," << bay << "," << baz << "\n";

    txt_file << std::fixed << std::setprecision(9)
             << timestamp.toSec() << " "
             << pos.x << " " << pos.y << " " << pos.z << " "
             << ori.x << " " << ori.y << " " << ori.z << " " << ori.w << "\n";
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "Usage: rosrun bag_creator bag_gt_extractor_node config.yaml" << std::endl;
        return 1;
    }

    rosbag::Bag bag;
    const std::string config_file_path = std::string(argv[1]);
    const YAML::Node config_node = YAML::LoadFile(config_file_path);
    const std::string input_bag_path = config_node["input_bag_path"].as<std::string>();
    const std::string bag_name = config_node["bag_name"].as<std::string>();
    const std::string bag_path = input_bag_path + bag_name;

    const std::string output_folder_path = config_node["output_folder_path"].as<std::string>();
    std::string output_name = "gt_data";
    if (config_node["output_name"]) {
        output_name = config_node["output_name"].as<std::string>();
    } else if (config_node["output_gt_prefix"]) {
        output_name = config_node["output_gt_prefix"].as<std::string>() + "gt_data";
    }
    if (output_name.empty()) {
        output_name = "gt_data";
    }

    bag.open(bag_path, rosbag::bagmode::Read);

    const std::string gt_topic = config_node["gt_topic"].as<std::string>();
    const int down_sample_ratio = config_node["down_sample_ratio"].as<int>();

    std::vector<std::string> topics;
    topics.push_back(gt_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    const fs::path output_folder(output_folder_path);
    std::ofstream csv_file((output_folder / (output_name + ".csv")).string());
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open CSV file for writing.\n";
        return 1;
    }

    csv_file << "#timestamp,"
             << "p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],"
             << "q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],"
             << "v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],"
             << "b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1],"
             << "b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]\n";

    std::ofstream txt_file((output_folder / (output_name + ".txt")).string());
    if (!txt_file.is_open()) {
        std::cerr << "Failed to open TXT file for writing.\n";
        return 1;
    }

    txt_file << "#timestamp x y z q_x q_y q_z q_w\n";

    std::vector<PoseRecord> pose_records;
    int value = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() != gt_topic && ("/" + m.getTopic() != gt_topic)) {
            continue;
        }

        geometry_msgs::PoseStamped::ConstPtr pose_msg = m.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg != nullptr) {
            value++;
            if (value % down_sample_ratio != 0) {
                continue;
            }

            const ros::Time timestamp = pose_msg->header.stamp;
            pose_records.push_back({timestamp, pose_msg->pose, isFinitePose(pose_msg->pose)});
            std::cout << timestamp << std::endl;
            continue;
        }

        nav_msgs::Path::ConstPtr path_msg = m.instantiate<nav_msgs::Path>();
        if (path_msg != nullptr) {
            for (const geometry_msgs::PoseStamped& pose_stamped : path_msg->poses) {
                value++;
                if (value % down_sample_ratio != 0) {
                    continue;
                }

                const ros::Time timestamp = pose_stamped.header.stamp;
                pose_records.push_back({timestamp, pose_stamped.pose, isFinitePose(pose_stamped.pose)});
                std::cout << value << "," << timestamp << std::endl;
            }
        }
    }

    int invalid_pose_count = 0;
    int interpolated_pose_count = 0;
    int unresolved_invalid_count = 0;
    int written_pose_count = 0;
    for (size_t i = 0; i < pose_records.size(); ++i) {
        geometry_msgs::Pose pose = pose_records[i].pose;
        if (!pose_records[i].valid) {
            invalid_pose_count++;
            if (!interpolatePoseFromNeighbors(pose_records, i, pose)) {
                unresolved_invalid_count++;
                continue;
            }
            interpolated_pose_count++;
        }

        writePose(csv_file, txt_file, pose_records[i].timestamp, pose);
        written_pose_count++;
    }

    csv_file.close();
    txt_file.close();

    std::cout << "GT extraction complete.\n";
    std::cout << "Total selected poses: " << pose_records.size() << "\n";
    std::cout << "Invalid input poses: " << invalid_pose_count << "\n";
    std::cout << "Interpolated poses: " << interpolated_pose_count << "\n";
    std::cout << "Unresolved invalid poses: " << unresolved_invalid_count << "\n";
    std::cout << "Poses written: " << written_pose_count << "\n";
    std::cout << output_name << ".csv created successfully.\n";

    bag.close();
    return 0;
}
