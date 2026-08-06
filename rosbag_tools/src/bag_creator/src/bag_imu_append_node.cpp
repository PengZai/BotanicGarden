#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

struct ImuSample {
    ros::Time stamp;
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
};

std::string trim(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t begin = value.find_first_not_of(whitespace);
    if (begin == std::string::npos) {
        return "";
    }

    const std::size_t end = value.find_last_not_of(whitespace);
    return value.substr(begin, end - begin + 1);
}

ros::Time nsToRosTime(const uint64_t timestamp_ns) {
    const uint32_t secs = static_cast<uint32_t>(timestamp_ns / 1000000000ULL);
    const uint32_t nsecs = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
    return ros::Time(secs, nsecs);
}

ros::Time timestampToRosTime(const uint64_t raw_timestamp) {
    const std::size_t digits = std::to_string(raw_timestamp).size();

    // Support common dataset timestamp units automatically.
    if (digits <= 10) {
        return ros::Time(static_cast<uint32_t>(raw_timestamp), 0);
    }
    if (digits <= 13) {
        return nsToRosTime(raw_timestamp * 1000000ULL);
    }
    if (digits <= 16) {
        return nsToRosTime(raw_timestamp * 1000ULL);
    }
    return nsToRosTime(raw_timestamp);
}

bool parseImuLine(const std::string& raw_line, ImuSample& sample) {
    std::string line = trim(raw_line);
    if (line.empty() || line[0] == '#') {
        return false;
    }

    std::replace(line.begin(), line.end(), ',', ' ');

    std::stringstream ss(line);
    uint64_t timestamp_ns = 0;
    if (!(ss >> timestamp_ns >> sample.wx >> sample.wy >> sample.wz >> sample.ax >> sample.ay >> sample.az)) {
        return false;
    }

    sample.stamp = timestampToRosTime(timestamp_ns);
    return true;
}

std::vector<ImuSample> loadImuSamples(const std::string& imu_file_path) {
    std::ifstream imu_file(imu_file_path);
    if (!imu_file.is_open()) {
        throw std::runtime_error("Failed to open IMU file: " + imu_file_path);
    }

    std::vector<ImuSample> samples;
    std::string line;
    while (std::getline(imu_file, line)) {
        ImuSample sample;
        if (parseImuLine(line, sample)) {
            samples.push_back(sample);
        }
    }

    std::sort(samples.begin(), samples.end(), [](const ImuSample& lhs, const ImuSample& rhs) {
        return lhs.stamp < rhs.stamp;
    });

    return samples;
}

void copyBagMessages(const std::string& input_bag_path, rosbag::Bag& outbag) {
    rosbag::Bag inbag;
    inbag.open(input_bag_path, rosbag::bagmode::Read);

    for (const rosbag::MessageInstance& m : rosbag::View(inbag)) {
        outbag.write(m.getTopic(), m.getTime(), m);
    }

    inbag.close();
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: rosrun bag_creator bag_imu_append_node config.yaml" << std::endl;
        return 1;
    }

    YAML::Node config = YAML::LoadFile(argv[1]);
    const std::string imu_file_path = config["imu_file_path"].as<std::string>();
    const std::string bag_path = config["bag_path"].as<std::string>();
    const std::string output_folder = config["output_folder_path"].as<std::string>();
    const std::string imu_topic = config["imu_topic"].as<std::string>();
    const std::string frame_id = config["frame_id"] ? config["frame_id"].as<std::string>() : "imu_link";

    const fs::path input_bag_path(bag_path);
    const fs::path output_dir(output_folder);
    const fs::path output_bag_path = output_dir / input_bag_path.filename();

    if (!fs::exists(output_dir)) {
        std::cerr << "Output folder does not exist: " << output_dir.string() << std::endl;
        return 1;
    }

    const std::vector<ImuSample> samples = loadImuSamples(imu_file_path);
    if (samples.empty()) {
        std::cerr << "No IMU samples found in: " << imu_file_path << std::endl;
        return 1;
    }

    const bool input_bag_exists = fs::exists(input_bag_path);
    const bool target_bag_exists = fs::exists(output_bag_path);

    rosbag::Bag outbag;
    if (target_bag_exists) {
        outbag.open(output_bag_path.string(), rosbag::bagmode::Append);
        std::cout << "Appending IMU data to existing bag: " << output_bag_path.string() << std::endl;
    } else {
        outbag.open(output_bag_path.string(), rosbag::bagmode::Write);
        if (input_bag_exists) {
            std::cout << "Copying existing bag to: " << output_bag_path.string() << std::endl;
            copyBagMessages(input_bag_path.string(), outbag);
        } else {
            std::cout << "Source bag not found. Creating new bag: " << output_bag_path.string() << std::endl;
        }
    }
    
    std::size_t written_count = 0;
    std::size_t total_size = samples.size();

    for (const ImuSample& sample : samples) {

        printf("Writing IMU sample at time: %u.%u, written: %zu/%zu\n", sample.stamp.sec, sample.stamp.nsec, written_count, total_size);

        sensor_msgs::Imu msg;
        msg.header.stamp = sample.stamp;
        msg.header.frame_id = frame_id;

        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        msg.orientation_covariance[0] = -1.0;

        msg.angular_velocity.x = sample.wx;
        msg.angular_velocity.y = sample.wy;
        msg.angular_velocity.z = sample.wz;

        msg.linear_acceleration.x = sample.ax;
        msg.linear_acceleration.y = sample.ay;
        msg.linear_acceleration.z = sample.az;
        outbag.write(imu_topic, sample.stamp, msg);
        written_count++;

    }

    outbag.close();
    std::cout << "Finished writing " << samples.size() << " IMU messages to: "
              << output_bag_path.string() << std::endl;
    return 0;
}
