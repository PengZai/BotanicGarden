#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>

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
    std::string output_gt_prefix = config_node["output_gt_prefix"].as<std::string>();
    std::string output_gt_format = config_node["output_gt_format"].as<std::string>();

    bag.open(bag_path, rosbag::bagmode::Read);

    std::string gt_topic = config_node["gt_topic"].as<std::string>();

    int down_sample_ratio = config_node["down_sample_ratio"].as<int>();

    // Image topics to load
    std::vector<std::string> topics;
    std::vector<std::pair<std::string, std::string>> pic_timestamps;
    topics.push_back(gt_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));


    std::ofstream csv_file(output_folder_path + "gt_data" + output_gt_format);
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open file for writing.\n";
        return 1;
    }

    // Write header
    csv_file << "#timestamp,"
    << "p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],"
    << "q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],"
    << "v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],"
    << "b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1],"
    << "b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]\n";

    const double vx = 0, vy = 0, vz =0;
    const double bwx = 0, bwy = 0, bwz = 0;
    const double bax = 0, bay = 0, baz = 0;


    int value = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view){
        if (m.getTopic() == gt_topic || ("/" + m.getTopic() == gt_topic))
        {
                value++;
                if (value % down_sample_ratio != 0){
                    continue;
                }
                const ros::Time time = m.getTime();
                std::cout << time << std::endl;
                geometry_msgs::PoseStamped::ConstPtr pose_msg = m.instantiate<geometry_msgs::PoseStamped>();

                if (pose_msg != NULL){
                    ros::Time timestamp = pose_msg->header.stamp;

                    const auto& pos = pose_msg->pose.position;
                    const auto& ori = pose_msg->pose.orientation;
                    
                    csv_file << timestamp.toNSec() << ","
                    << pos.x << "," << pos.y << "," << pos.z << ","
                    << ori.w << "," << ori.x << "," << ori.y << "," << ori.z << ","
                    << vx << "," << vy << "," << vz << ","
                    << bwx << "," << bwy << "," << bwz << ","
                    << bax << "," << bay << "," << baz << "\n";
           
            }
        }
    }



  
    csv_file.close();
    std::cout << "gt_data.csv created successfully.\n";
    
    bag.close();

    return 0;
}