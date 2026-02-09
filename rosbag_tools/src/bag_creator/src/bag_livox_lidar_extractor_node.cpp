#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <livox_ros_driver/CustomMsg.h>
#include <boost/foreach.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

int main(int argc, char **argv)
{
    rosbag::Bag bag;
    std::string config_file_path = std::string(argv[1]);
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::string input_bag_path = config_node["input_bag_path"].as<std::string>();
    std::string bag_name = config_node["bag_name"].as<std::string>();
    std::string bag_path = input_bag_path + bag_name;

    std::string lidar_topic = config_node["lidar_topic"].as<std::string>();
    std::string output_folder_path = config_node["output_folder_path"].as<std::string>();
    int down_sample_ratio = config_node["down_sample_ratio"].as<int>();

    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics = {lidar_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int count = 0;

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == lidar_topic || ("/" + m.getTopic() == lidar_topic)) {
            count++;
            if (count % down_sample_ratio != 0)
                continue;

            livox_ros_driver::CustomMsg::ConstPtr livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
            if (livox_msg != NULL) {
                pcl::PointCloud<pcl::PointXYZI> cloud;

                for (const auto& pt : livox_msg->points) {
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
