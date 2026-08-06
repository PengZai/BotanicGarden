#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <string>

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
    std::string output_image_prefix = config_node["output_image_prefix"].as<std::string>();
    std::string output_image_format = config_node["output_image_format"].as<std::string>();

    bag.open(bag_path, rosbag::bagmode::Read);

    std::string l_cam = config_node["cam_topic"].as<std::string>();

    int down_sample_ratio = config_node["down_sample_ratio"].as<int>();

    // Image topics to load
    std::vector<std::string> topics;
    std::vector<std::pair<std::string, std::string>> pic_timestamps;
    std::vector<std::pair<std::string, std::string>> image_name_timestamps;

    topics.push_back(l_cam);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // ros::Time bag_begin_time = view.getBeginTime();
    // ros::Time bag_end_time = view.getEndTime();

    // std::cout << "ROS bag time: " << (bag_end_time-bag_begin_time).toSec() << "(s)" << std::endl;
    int value = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == l_cam || ("/" + m.getTopic() == l_cam)) {
            value++;

            if (value % down_sample_ratio != 0) {
                continue;
            }

            const ros::Time time = m.getTime();
            std::cout << value << ") " <<"Bag Time: " << time << std::endl;

            cv::Mat image;

            // Try raw Image
            sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
            if (l_img) {
                try {
                    image = cv_bridge::toCvCopy(l_img, "bgr8")->image;
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    continue;
                }
            }

            // Try compressed Image
            sensor_msgs::CompressedImage::ConstPtr l_img_comp = m.instantiate<sensor_msgs::CompressedImage>();
            if (l_img_comp) {
                try {
                    image = cv::imdecode(cv::Mat(l_img_comp->data), cv::IMREAD_COLOR);
                } catch (cv::Exception& e) {
                    ROS_ERROR("cv::imdecode exception: %s", e.what());
                    continue;
                }
            }

            if (!image.empty()) {
                ros::Time ros_timestamp = (l_img ? l_img->header.stamp : l_img_comp->header.stamp);

                std::string str_time = std::to_string(ros_timestamp.toNSec());
                // std::string time_in_seconds = std::to_string(ros_timestamp.sec) + "." + std::to_string(ros_timestamp.nsec);
                std::string time_in_seconds = str_time.substr(0, 10) + "." + str_time.substr(10);
                std::cout << value << ") " << "Source Time: " << time_in_seconds << std::endl;

                std::string image_name = str_time + output_image_format;
                pic_timestamps.push_back(std::make_pair(str_time, image_name));
                image_name_timestamps.push_back(std::make_pair(str_time, time_in_seconds));
                std::string file_name = output_folder_path + output_image_prefix + image_name;

                cv::imwrite(file_name, image);
            }
        }
    }


    std::string replaced_l_cam = l_cam; 
    std::replace(replaced_l_cam.begin(), replaced_l_cam.end(), '/', '_');

    std::ofstream csv_file(output_folder_path + replaced_l_cam + "_pic_timestamps.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open file for writing.\n";
        return 1;
    }

    // Write header
    csv_file << "#timestamp [ns],filename\n";

    // Write data
    for (const auto& entry : pic_timestamps) {
        csv_file << entry.first << "," << entry.second << "\n";
    }

    csv_file.close();
    std::cout << "pic_timestamps.csv created successfully.\n";


    std::ofstream txt_file(output_folder_path + replaced_l_cam + "_image_name_timestamps.txt");
    if (!txt_file.is_open()) {
        std::cerr << "Failed to open file for writing.\n";
        return 1;
    }

    // Write header
    // txt_file << "#timestamp [ns],filename\n";

    // Write data
    for (const auto& entry : image_name_timestamps) {
        txt_file << entry.first << " " << entry.second << "\n";
    }

    txt_file.close();
    std::cout << "_image_name_timestamps.txt created successfully.\n";
    
    bag.close();

    return 0;
}