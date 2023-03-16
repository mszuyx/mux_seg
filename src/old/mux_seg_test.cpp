// Import ROS lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


class MuxSeg{
public:
    MuxSeg();
private:
    // Declare sub & pub
    ros::NodeHandle node_handle_;
   
    ros::Subscriber range1_node_sub_;
    ros::Subscriber range2_node_sub_;
    ros::Subscriber lidar1_node_sub_;
    ros::Subscriber lidar2_node_sub_;


    // Declare functions
    // void secondaryCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, const std::string &topic);
    void secondaryCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud);
 
};

MuxSeg::MuxSeg():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing Ground Plane Segmentation Node...");


    // Subscribe to secondary sensors
    range1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/evo_mini_0/pointcloud", 5, &MuxSeg::secondaryCallback_, this); //5
    // range1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/evo_mini_0/pointcloud", 5, boost::bind(&MuxSeg::secondaryCallback_, this, _1, "range_in_right")); //5
    // range2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/range_in_left", 5, boost::bind(&MuxSeg::secondaryCallback_, this, _1, "range_in_left")); //5
    // lidar1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_in_right", 5, boost::bind(&MuxSeg::secondaryCallback_, this, _1, "lidar_in_right")); //5
    // lidar2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_in_left", 5, boost::bind(&MuxSeg::secondaryCallback_, this, _1, "lidar_in_left")); //5
}

void MuxSeg::secondaryCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud){
    ROS_INFO("callback"); 
}

// void MuxSeg::secondaryCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, const std::string &topic){
//     ROS_INFO("callback"); 
//     if(topic == "range_in_right"){
//         std::cout<< "range_in_right" <<std::endl;
//     }else if(topic == "range_in_left"){
//         std::cout<< "range_in_left" <<std::endl;
//     }else if(topic == "lidar_in_right"){
//         std::cout<< "lidar_in_right" <<std::endl;
//     }else if(topic == "lidar_in_left"){
//         std::cout<< "lidar_in_left" <<std::endl;
//     }else{
//         std::cout<< "Unknown topic name" <<std::endl;
//     }

// }


int main (int argc, char** argv) {
    ros::init(argc, argv, "MuxSeg");
    MuxSeg node;
    // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    // spinner.spin();
    ros::spin();
    return 0;
 }
