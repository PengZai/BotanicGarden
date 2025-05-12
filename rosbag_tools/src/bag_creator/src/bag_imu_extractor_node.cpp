#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <sensor_msgs/Imu.h>

#include <yaml-cpp/yaml.h>



int main(int argc, char **argv)
{
    rosbag::Bag bag;
    std::string config_file_path = std::string(argv[1]);
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    std::string input_bag_path = config_node["input_bag_path"].as<std::string>();
    std::string bag_name = config_node["bag_name"].as<std::string>();
    std::string bag_path = input_bag_path + bag_name;

    std::string output_folder_path = config_node["output_folder_path"].as<std::string>();
    std::string output_imu_prefix = config_node["output_imu_prefix"].as<std::string>();
    std::string output_imu_format = config_node["output_imu_format"].as<std::string>();

    bag.open(bag_path, rosbag::bagmode::Read);

    std::string imu_topic = config_node["imu_topic"].as<std::string>();

    int down_sample_ratio = config_node["down_sample_ratio"].as<int>();

    // Image topics to load
    std::vector<std::string> topics;
    std::vector<std::pair<std::string, std::string>> pic_timestamps;
    topics.push_back(imu_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));


    std::ofstream csv_file(output_folder_path + "imu_data" + output_imu_format);
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open file for writing.\n";
        return 1;
    }

    // Write header
    csv_file << "#timestamp [ns],"
    << "w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
    << "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";

    int value = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view){
        if (m.getTopic() == imu_topic || ("/" + m.getTopic() == imu_topic))
        {
                value++;
                if (value % down_sample_ratio != 0){
                    continue;
                }
                const ros::Time time = m.getTime();
                std::cout << time << std::endl;
                sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();

                if (imu != NULL){
                    ros::Time timestamp = imu->header.stamp;

                    double wx = imu->angular_velocity.x;
                    double wy = imu->angular_velocity.y;
                    double wz = imu->angular_velocity.z;

                    double ax = imu->linear_acceleration.x;
                    double ay = imu->linear_acceleration.y;
                    double az = imu->linear_acceleration.z;
                    
                    csv_file << timestamp.toNSec() << ","
                    << wx << "," << wy << "," << wz << ","
                    << ax << "," << ay << "," << az << "\n";
            }
        }
    }



  
    csv_file.close();
    std::cout << "imu_data.csv created successfully.\n";
    
    bag.close();

    return 0;
}