#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    std::string config_file_path = std::string(argv[1]);
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::string input_bag_path = config_node["input_bag_path"].as<std::string>();
    std::string bag_name = config_node["bag_name"].as<std::string>();
    std::string bag_path = input_bag_path + bag_name;

    std::string lidar_topic = config_node["lidar_topic"].as<std::string>();
    std::string output_folder_path = config_node["output_folder_path"].as<std::string>();
    int down_sample_ratio = config_node["down_sample_ratio"].as<int>();

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics = {lidar_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int count = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == lidar_topic || ("/" + m.getTopic() == lidar_topic)) {
            count++;
            if (count % down_sample_ratio != 0)
                continue;

            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg != nullptr) {
                pcl::PointCloud<pcl::PointXYZI> cloud;
                pcl::fromROSMsg(*cloud_msg, cloud);

                std::string filename = output_folder_path +
                    std::to_string(cloud_msg->header.stamp.toNSec()) + ".pcd";

                pcl::io::savePCDFileBinary(filename, cloud);
                std::cout << "Saved: " << filename << std::endl;
            }
        }
    }

    bag.close();
    return 0;
}
