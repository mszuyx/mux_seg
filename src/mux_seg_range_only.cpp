// Import ROS lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// Import PCL lib
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/passthrough.h>
// Import Eigen lib
#include <Eigen/Dense> 
// Import message_filters lib
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Import tf
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <mutex> 

using namespace message_filters;
using namespace Eigen;

// Shared pointcloud variables
sensor_msgs::PointCloud2::Ptr range_cloud_right_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr range_cloud_left_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr lidar_cloud_right_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr lidar_cloud_left_transformed(new sensor_msgs::PointCloud2 ());

std::mutex mutex_;

class MuxSeg_range{
public:
    MuxSeg_range();
private:
    // Declare sub & pub
    ros::NodeHandle node_handle_;
    ros::Publisher range_points_pub_;
    ros::Publisher gp_param_pub_;
    tf::TransformListener tfListener;

    // Declare ROS params
    double th_ceil_;
    double th_box_;
    double dead_zone_;
    double map_unit_size_;
    std::string base_FrameId;
    std::string right_range_FrameId;
    std::string left_range_FrameId;
    std::string right_lidar_FrameId;
    std::string left_lidar_FrameId;
    bool compress_z;

    ros::Subscriber range1_node_sub_;
    ros::Subscriber range2_node_sub_;
    ros::Subscriber lidar1_node_sub_;
    ros::Subscriber lidar2_node_sub_;
    ros::Timer timer;


    // Declare functions
    void merge_point32_vector(const std::vector<geometry_msgs::Point32>& vin, std::vector<geometry_msgs::Point32>& vout);
    void secondaryCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, const std::string &topic);
    void timerCallback_ ();
    bool get_trans_matrix(const std::string &parentFrame, const std::string &childFrame, Matrix4f& tfMatrix);

    // Matrix4f cam_r_2_cam_r_Matrix;
    // bool get_cam_r_2_cam_r_Matrix = false;
    Matrix4f cam_l_2_cam_r_Matrix;
    bool get_cam_l_2_cam_r_Matrix = false;
    Matrix4f range_r_2_base_Matrix;
    bool get_range_r_2_base_Matrix = false;
    Matrix4f range_l_2_base_Matrix;
    bool get_range_l_2_base_Matrix = false;
    Matrix4f lidar_r_2_base_Matrix;
    bool get_lidar_r_2_base_Matrix = false;
    Matrix4f lidar_l_2_base_Matrix;
    bool get_lidar_l_2_base_Matrix= false;
};

MuxSeg_range::MuxSeg_range():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing mux segmentation range only Node...");

    node_handle_.param("th_ceil_", th_ceil_, 1.0);
    ROS_INFO("Ceiling Threshold: %f", th_ceil_);

    node_handle_.param("th_box_", th_box_, 7.0);
    ROS_INFO("Box Threshold: %f", th_box_);

    node_handle_.param("dead_zone_", dead_zone_, 0.3);
    ROS_INFO("Sensor deadzone: %f", dead_zone_);

    node_handle_.param("map_unit_size_", map_unit_size_, 0.15); 
    ROS_INFO("map_unit_size_: %f", map_unit_size_);

    node_handle_.param<std::string>("base_frame_id", base_FrameId, "/base_link");
    ROS_INFO("base frame_id: %s", base_FrameId.c_str());

    node_handle_.param<std::string>("right_range_frame_id", right_range_FrameId, "/right_range_link");
    ROS_INFO("right range frame_id: %s", right_range_FrameId.c_str());
    node_handle_.param<std::string>("left_range_frame_id", left_range_FrameId, "/left_range_link");
    ROS_INFO("left range frame_id: %s", left_range_FrameId.c_str());

    node_handle_.param<std::string>("right_lidar_frame_id", right_lidar_FrameId, "/right_lidar_link");
    ROS_INFO("right lidar frame_id: %s", right_lidar_FrameId.c_str());
    node_handle_.param<std::string>("left_lidar_frame_id", left_lidar_FrameId, "/left_lidar_link");
    ROS_INFO("left lidar frame_id: %s", left_lidar_FrameId.c_str());

    node_handle_.param("compress_z", compress_z, false);
    ROS_INFO("Compress the z-axis reading for point cloud?: %d", compress_z);

    // Subscribe to secondary sensors
    range1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/range_in_right", 1, boost::bind(&MuxSeg_range::secondaryCallback_, this, _1, "range_in_right")); //5
    range2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/range_in_left", 1, boost::bind(&MuxSeg_range::secondaryCallback_, this, _1, "range_in_left")); //5
    lidar1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_in_right", 1, boost::bind(&MuxSeg_range::secondaryCallback_, this, _1, "lidar_in_right")); //5
    lidar2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_in_left", 1, boost::bind(&MuxSeg_range::secondaryCallback_, this, _1, "lidar_in_left")); //5

    timer = node_handle_.createTimer(ros::Duration(0.025), boost::bind(&MuxSeg_range::timerCallback_,this));

    // Publish Init
    std::string range_points_topic;
    node_handle_.param<std::string>("range_points_topic", range_points_topic, "/range_points");
    ROS_INFO("range_points_topic is published at: %s", range_points_topic.c_str());
    range_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(range_points_topic, 1);
}


void MuxSeg_range::merge_point32_vector(const std::vector<geometry_msgs::Point32>& vin, std::vector<geometry_msgs::Point32>& vout){
    vout.reserve(vout.size() + vin.size());
    vout.insert(vout.end(), vin.begin(), vin.end());
}

bool MuxSeg_range::get_trans_matrix(const std::string &parentFrame, const std::string &childFrame, Matrix4f& tfMatrix){
    tf::StampedTransform Tf;
    try {
        ros::Time now = ros::Time::now();
        tfListener.waitForTransform(parentFrame, childFrame, now, ros::Duration(2.0));
        tfListener.lookupTransform(parentFrame, childFrame, now, Tf);
        pcl_ros::transformAsMatrix(Tf, tfMatrix);
        ROS_INFO("Got tf matrix!"); 
        std::cout<< tfMatrix <<std::endl;
        return true;
    } 
    catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of camera data: " << ex.what() << ", quitting callback");
        return false;
    }
}

void MuxSeg_range::secondaryCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, const std::string &topic){
    {const std::lock_guard<std::mutex> lock(mutex_);
        // ROS_INFO("callback"); 
        if(topic == "range_in_right"){
            if(get_range_r_2_base_Matrix==false){
                get_range_r_2_base_Matrix = get_trans_matrix(base_FrameId, right_range_FrameId, range_r_2_base_Matrix);
                std::cout<< right_range_FrameId <<std::endl;
            }else{pcl_ros::transformPointCloud(range_r_2_base_Matrix, *input_cloud, *range_cloud_right_transformed);}
            
        }else if(topic == "range_in_left"){
            if(get_range_l_2_base_Matrix==false){
                get_range_l_2_base_Matrix = get_trans_matrix(base_FrameId, left_range_FrameId, range_l_2_base_Matrix);
                std::cout<< left_range_FrameId <<std::endl;
            }else{pcl_ros::transformPointCloud(range_l_2_base_Matrix, *input_cloud, *range_cloud_left_transformed);}

        }else if(topic == "lidar_in_right"){
            if(get_lidar_r_2_base_Matrix==false){
                get_lidar_r_2_base_Matrix = get_trans_matrix(base_FrameId, right_lidar_FrameId, lidar_r_2_base_Matrix);
                std::cout<< right_lidar_FrameId <<std::endl;
            }else{pcl_ros::transformPointCloud(lidar_r_2_base_Matrix, *input_cloud, *lidar_cloud_right_transformed);}
            
        }else if(topic == "lidar_in_left"){
            if(get_lidar_l_2_base_Matrix==false){
                get_lidar_l_2_base_Matrix = get_trans_matrix(base_FrameId, left_lidar_FrameId, lidar_l_2_base_Matrix);
                std::cout<< left_lidar_FrameId <<std::endl;
            }else{pcl_ros::transformPointCloud(lidar_l_2_base_Matrix, *input_cloud, *lidar_cloud_left_transformed);}
            
        }else{
            ROS_INFO("Unknown topic name"); 
        }
    }
}


void MuxSeg_range::timerCallback_ (){
    // ROS_INFO("callback"); 

    // 1. Create global PC
    pcl::PCLPointCloud2::Ptr cloud_pcl (new pcl::PCLPointCloud2 ());

    // 2. Merge all other PCs
    if(get_range_r_2_base_Matrix==true){
        pcl::PCLPointCloud2::Ptr range_cloud_right_pcl (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*range_cloud_right_transformed, *range_cloud_right_pcl);
        // pcl::removeNaNFromPointCloud(*range_cloud_right_pcl,*range_cloud_right_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*range_cloud_right_pcl);
    }
    if(get_range_l_2_base_Matrix==true){
        pcl::PCLPointCloud2::Ptr range_cloud_left_pcl (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*range_cloud_left_transformed, *range_cloud_left_pcl);
        // pcl::removeNaNFromPointCloud(*range_cloud_left_pcl,*range_cloud_left_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*range_cloud_left_pcl);
    }
    if(get_lidar_r_2_base_Matrix==true){
        pcl::PCLPointCloud2::Ptr lidar_cloud_right_pcl (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*lidar_cloud_right_transformed, *lidar_cloud_right_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*lidar_cloud_right_pcl);
    }
    if(get_lidar_l_2_base_Matrix==true){
        pcl::PCLPointCloud2::Ptr lidar_cloud_left_pcl (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*lidar_cloud_left_transformed, *lidar_cloud_left_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*lidar_cloud_left_pcl);
    }

    // 3.Convert comb pc to pcl::PointXYZ
    if(cloud_pcl->width > 0){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::fromPCLPointCloud2(*cloud_pcl, *cloud_raw);

        // 5.Clip based on box threshold.
        pcl::CropBox<pcl::PointXYZ> box;
        box.setMin(Vector4f(-th_box_, -th_box_, -1.0,   1.0));
        box.setMax(Vector4f( th_box_,  th_box_, th_ceil_,   1.0));
        box.setInputCloud(cloud_raw);
        box.filter(*cloud_raw);

        // pcl::CropBox<pcl::PointXYZ> in_bound;
        box.setMin(Vector4f(-dead_zone_-0.1, -dead_zone_, -0.3,   1.0));
        box.setMax(Vector4f( dead_zone_+0.2,  dead_zone_,  0.0,   1.0));
        box.setInputCloud(cloud_raw);
        box.setNegative (true);
        box.filter(*cloud_raw);
        // pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(*cloud_raw, *cloud);

        if(compress_z){
            for(size_t i=0 ; i < cloud_raw->points.size(); i++){
                (*cloud_raw)[i].z = 0.0;
            }
        }

        //publish not ground points
        sensor_msgs::PointCloud2::Ptr pc_msg (new sensor_msgs::PointCloud2 ());
        pcl::toROSMsg(*cloud_raw, *pc_msg);
        pc_msg->header.stamp = ros::Time::now();
        pc_msg->header.frame_id = base_FrameId;
        range_points_pub_.publish(*pc_msg);
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "MuxSeg_range");
    MuxSeg_range node;
    ros::MultiThreadedSpinner spinner(5); // Use multi threads
    spinner.spin();
    // ros::spin();
    return 0;
 }
