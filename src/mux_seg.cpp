// Import ROS lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
// Import PCL lib
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
// Import Eigen lib
#include <Eigen/Dense> 
// Import message_filters lib
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// Custom msg type
#include <pc_gps/gpParam.h>
// Import tf
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_ros/transforms.h>
// #include <mutex> 

// using namespace message_filters;
using namespace Eigen;

// Declare helper functions
bool point_cmp(const pcl::PointXYZ a, const pcl::PointXYZ b){return a.z<b.z;}

// Declare pc variables
pcl::PointCloud<pcl::PointXYZ>::Ptr seeds_pc(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pc(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr not_ground_pc(new pcl::PointCloud<pcl::PointXYZ>());

// Shared pointcloud variables
sensor_msgs::PointCloud2::Ptr camera_cloud_right_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr camera_cloud_left_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr range_cloud_right_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr range_cloud_left_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr lidar_cloud_right_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr lidar_cloud_left_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::Imu::Ptr imu_raw(new sensor_msgs::Imu ());

// std::mutex mutex_;
double last_d_;

class MuxSeg{
public:
    MuxSeg();
private:
    // Declare sub & pub
    ros::NodeHandle node_handle_;
    ros::Publisher ground_points_pub_;
    ros::Publisher groundless_points_pub_;
    ros::Publisher gp_param_pub_;
    tf::TransformListener tfListener;

    ros::Publisher VGF_pub_;
    ros::Publisher OA_pub_; 
    ros::Publisher BC_pub_; 
    ros::Publisher ROR_pub_;
    ros::Publisher SEED_pub_;
    ros::Publisher normal_vec_pub_;
    ros::Publisher imu_vec_pub_;

    // Declare ROS params
    int num_iter_, in_radius_;
    double sensor_height_, th_seeds_, th_dist_, th_ceil_, th_box_, dead_zone_, map_unit_size_, block_thres_;
    double num_lpr_, radius_search_;
    double roll_offset_, pitch_offset_;
    bool SVD_refinement, SVD_holdoff, detect_neg, compress_z;
    bool debug, timer;
    std::string base_FrameId;
    std::string right_camera_FrameId;
    std::string left_camera_FrameId;
    std::string right_range_FrameId;
    std::string left_range_FrameId;
    std::string right_lidar_FrameId;
    std::string left_lidar_FrameId;

    ros::Subscriber imu_node_sub_;
    ros::Subscriber camera1_node_sub_;
    ros::Subscriber camera2_node_sub_;
    ros::Subscriber range1_node_sub_;
    ros::Subscriber range2_node_sub_;
    ros::Subscriber lidar1_node_sub_;
    ros::Subscriber lidar2_node_sub_;
    ros::Timer timer_node_;

    // Declare functions
    void timerCallback_ ();
    // void rs_pc_callback_ (const sensor_msgs::PointCloud2::ConstPtr& input_cloud_right, const sensor_msgs::PointCloud2::ConstPtr& input_cloud_left);
    void sensorCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, const std::string &topic);
    void imuCallback_(const sensor_msgs::Imu::ConstPtr& msg);
    void merge_point32_vector(const std::vector<geometry_msgs::Point32>& vin, std::vector<geometry_msgs::Point32>& vout);
    bool get_trans_matrix(const std::string &parentFrame, const std::string &childFrame, Matrix4f& tfMatrix);
    void quaternionToMatrix(double q0, double q1, double q2, double q3,  Affine3d& transform);
    int findMean(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, double& mean, const double percentage);
    void estimate_plane_(void);
    bool checkBlock(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, const double threshold);
    void extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZ>::Ptr& p_sorted);

    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0, here normal:=[a,b,c], d=d, th_dist_d_ = threshold_dist - d 
    double d_;
    Vector3d normal_imu;
    Vector3d normal_;
    double th_dist_d_;
    double last_lpr;

    Matrix4f cam_r_2_base_Matrix;
    bool get_cam_r_2_base_Matrix = false;
    Matrix4f cam_l_2_base_Matrix;
    bool get_cam_l_2_base_Matrix = false;
    Matrix4f range_r_2_base_Matrix;
    bool get_range_r_2_base_Matrix = false;
    Matrix4f range_l_2_base_Matrix;
    bool get_range_l_2_base_Matrix = false;
    Matrix4f lidar_r_2_base_Matrix;
    bool get_lidar_r_2_base_Matrix = false;
    Matrix4f lidar_l_2_base_Matrix;
    bool get_lidar_l_2_base_Matrix= false;
};

MuxSeg::MuxSeg():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing mux ground plane segmentation Node...");

    node_handle_.param("th_ceil_", th_ceil_, 1.0);
    ROS_INFO("Ceiling Threshold: %f", th_ceil_);

    node_handle_.param("th_box_", th_box_, 7.0);
    ROS_INFO("Box Threshold: %f", th_box_);

    node_handle_.param("dead_zone_", dead_zone_, 0.3);
    ROS_INFO("Sensor deadzone: %f", dead_zone_);

    node_handle_.param("map_unit_size_", map_unit_size_, 0.15); 
    ROS_INFO("map_unit_size_: %f", map_unit_size_);

    node_handle_.param("sensor_height", sensor_height_, 1.0);
    ROS_INFO("Sensor Height: %f", sensor_height_);

    node_handle_.param("num_iter", num_iter_, 3);
    ROS_INFO("Num of Iteration: %d", num_iter_);

    node_handle_.param("num_lpr", num_lpr_, 0.9);
    ROS_INFO("Num of LPR: %f", num_lpr_);

    node_handle_.param("th_seeds", th_seeds_, 0.05);
    ROS_INFO("Seeds Threshold: %f", th_seeds_);

    node_handle_.param("th_dist", th_dist_, 0.02);
    ROS_INFO("Distance Threshold: %f", th_dist_);

    node_handle_.param("block_thres_", block_thres_, 0.4);
    ROS_INFO("Blockage Threshold: %f", block_thres_);

    node_handle_.param("radius_search_", radius_search_, 0.15);
    ROS_INFO("radius_search_: %f", radius_search_);

    node_handle_.param("in_radius_", in_radius_, -25);
    ROS_INFO("in_radius_: %d", in_radius_);

    node_handle_.param("SVD_refinement", SVD_refinement, false);
    ROS_INFO("Do SVD refinement?: %d", SVD_refinement);

    node_handle_.param("roll_offset_", roll_offset_, 0.0); 
    ROS_INFO("roll_offset_: %f", roll_offset_);

    node_handle_.param("pitch_offset_", pitch_offset_, 0.0); 
    ROS_INFO("pitch_offset_: %f", pitch_offset_);

    node_handle_.param("debug", debug, false);
    ROS_INFO("Enter debug mode?: %d", debug);

    node_handle_.param("timer", timer, false);
    ROS_INFO("Enter debug mode?: %d", timer);

    node_handle_.param<std::string>("base_frame_id", base_FrameId, "/base_link");
    ROS_INFO("base frame_id: %s", base_FrameId.c_str());

    node_handle_.param<std::string>("right_camera_frame_id", right_camera_FrameId, "/right_camera_link");
    ROS_INFO("right camera frame_id: %s", right_camera_FrameId.c_str());
    node_handle_.param<std::string>("left_camera_frame_id", left_camera_FrameId, "/left_camera_link");
    ROS_INFO("left camera frame_id: %s", left_camera_FrameId.c_str());

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

    // Subscribe to sensors
    camera1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/camera_in_right", 1, boost::bind(&MuxSeg::sensorCallback_, this, _1, "camera_in_right")); //5
    camera2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/camera_in_left", 1, boost::bind(&MuxSeg::sensorCallback_, this, _1, "camera_in_left")); //5
    range1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/range_in_right", 1, boost::bind(&MuxSeg::sensorCallback_, this, _1, "range_in_right")); //5
    range2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/range_in_left", 1, boost::bind(&MuxSeg::sensorCallback_, this, _1, "range_in_left")); //5
    lidar1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_in_right", 1, boost::bind(&MuxSeg::sensorCallback_, this, _1, "lidar_in_right")); //5
    lidar2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_in_left", 1, boost::bind(&MuxSeg::sensorCallback_, this, _1, "lidar_in_left")); //5
    imu_node_sub_ = node_handle_.subscribe<sensor_msgs::Imu>("/imu_data", 1, &MuxSeg::imuCallback_, this); 
    timer_node_ = node_handle_.createTimer(ros::Duration(0.033), boost::bind(&MuxSeg::timerCallback_,this));

    // Publish Init
    std::string not_ground_point_topic, ground_topic, gp_param_topic;
    node_handle_.param<std::string>("not_ground_point_topic", not_ground_point_topic, "/gp_segmentation/cloud/not_ground");
    ROS_INFO("Not Ground Output Point Cloud: %s", not_ground_point_topic.c_str());
    groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(not_ground_point_topic, 1);
    node_handle_.param<std::string>("ground_point_topic", ground_topic, "/gp_segmentation/cloud/ground");
    ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());
    ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 1);
    node_handle_.param<std::string>("groundplane_param_topic", gp_param_topic, "/gp_segmentation/param");
    ROS_INFO("Ground plane parameters topic: %s", gp_param_topic.c_str());
    gp_param_pub_ = node_handle_.advertise<pc_gps::gpParam>(gp_param_topic, 1);

    normal_ << 0.0,0.0,1.0;
    last_d_ = sensor_height_;
    d_ = sensor_height_;

    if(debug){
    std::string VGF_topic, OA_topic, BC_topic, ROR_topic, SEED_topic, normal_topic, imu_normal_topic;
    node_handle_.param<std::string>("VGF_topic", VGF_topic, "/gp_segmentation/cloud/VGF");
    VGF_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(VGF_topic, 1);
    node_handle_.param<std::string>("OA_topic", OA_topic, "/gp_segmentation/cloud/OA");
    OA_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(OA_topic, 1);
    node_handle_.param<std::string>("BC_topic", BC_topic, "/gp_segmentation/cloud/BC");
    BC_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(BC_topic, 1);
    node_handle_.param<std::string>("ROR_topic", ROR_topic, "/gp_segmentation/cloud/ROR");
    ROR_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ROR_topic, 1);
    node_handle_.param<std::string>("SEED_topic", SEED_topic, "/gp_segmentation/cloud/SEED");
    SEED_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(SEED_topic, 1);

    node_handle_.param<std::string>("normal_topic", normal_topic, "/gp_segmentation/normal");
    normal_vec_pub_ = node_handle_.advertise<visualization_msgs::Marker>(normal_topic, 1);
    node_handle_.param<std::string>("imu_normal_topic", imu_normal_topic, "/gp_segmentation/imu_normal");
    imu_vec_pub_ = node_handle_.advertise<visualization_msgs::Marker>(imu_normal_topic, 1);
    }
}


void MuxSeg::merge_point32_vector(const std::vector<geometry_msgs::Point32>& vin, std::vector<geometry_msgs::Point32>& vout){
    vout.reserve(vout.size() + vin.size());
    vout.insert(vout.end(), vin.begin(), vin.end());
}

bool MuxSeg::get_trans_matrix(const std::string &parentFrame, const std::string &childFrame, Matrix4f& tfMatrix){
    tf::StampedTransform Tf;
    try {
        // ros::Time now = ros::Time::now();
        // tfListener.waitForTransform(parentFrame, childFrame, now, ros::Duration(2.0));
        // tfListener.lookupTransform(parentFrame, childFrame, now, Tf);
        tfListener.lookupTransform(parentFrame, childFrame, ros::Time(0), Tf);
        pcl_ros::transformAsMatrix(Tf, tfMatrix);
        ROS_INFO("Got tf matrix!"); 
        std::cout<< childFrame <<std::endl;
        std::cout<< tfMatrix <<std::endl;
        return true;
    } 
    catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of camera data: " << ex.what() << ", quitting callback");
        return false;
    }
}

void MuxSeg::quaternionToMatrix(double q0, double q1, double q2, double q3, Affine3d& transform){
    double t0 = 2 * (q0 * q1 + q2 * q3);
    double t1 = 1 - 2 * (q1 * q1 + q2 * q2);
    double roll = std::atan2(t0, t1);

    double t2 = 2 * (q0 * q2 - q3 * q1);
    if (t2 >= 1){t2 = 1.0;}
    else if (t2<= -1){t2 = -1.0;}
    double pitch = std::asin(t2);

    //double t3 = 2 * (q0 * q3 + q1 * q2);
    //double t4 = 1 - 2 * (q2 * q2 + q3 * q3);
    //double yaw = std::atan2(t3, t4);

    // axis defined in camera_depth_optical_frame
    transform = AngleAxisd(pitch+pitch_offset_, Vector3d::UnitY()) * AngleAxisd(roll+roll_offset_, Vector3d::UnitX()); 
}

int MuxSeg::findMean(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, double& mean, const double percentage){
    int cnt = 0;
    if (percentage>0){
        for(size_t i=0;i<pc->points.size() && cnt<int(percentage*pc->points.size());i++){
            mean += double(pc->points[i].z);
            cnt++;
        }
    }else{
        for(size_t i=0;i<pc->points.size();i++){
            mean += double(pc->points[i].z);
            cnt++;
        }
    }
    mean = cnt!=0?mean/cnt:0;
    return cnt;
}

void MuxSeg::estimate_plane_(void){
    if(SVD_refinement==true && SVD_holdoff==false){
        // Create covarian matrix in single pass.
        Matrix3d cov;
        Vector4d pc_mean;
        pcl::computeMeanAndCovarianceMatrix(*ground_pc, cov, pc_mean);
        // Singular Value Decomposition: SVD
        JacobiSVD<MatrixXd> svd(cov,DecompositionOptions::ComputeFullU);
        // use the least singular vector as normal
        normal_ = (svd.matrixU().col(2));
        // mean ground seeds value
        Vector3d seeds_mean = pc_mean.head<3>();
        //according to normal.T*[x,y,z] = -d
        d_ = (normal_.transpose()*seeds_mean)(0,0);
    }else{
        double pc_mean_z = 0.0;
        findMean(ground_pc, pc_mean_z, -1);
        d_ = pc_mean_z;
    }
    // set distance threhold to `th_dist + d`
    th_dist_d_ = th_dist_ + d_;
    // update the equation parameters
}

bool MuxSeg::checkBlock(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, const double threshold){
    if(pc->points.size()<100){return true;}
    double mean_x = 0.0;
    int cnt = 0;
    for(size_t i=0;i<pc->points.size();i++){
        mean_x += double(pc->points[i].x);
        cnt++;
    }
    mean_x = cnt!=0?mean_x/cnt:0;
    if(mean_x<threshold){
        return true;
    }else{
        return false;
    }
}

void MuxSeg::extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZ>::Ptr& p_sorted){
    // LPR is the mean of low point representative
    // Calculate the mean height value.
    double lpr_height = 0.0;
    int cnt = findMean(p_sorted, lpr_height, num_lpr_);
    if(cnt<300){
        SVD_holdoff = true;
    }else{
        SVD_holdoff = false;
    }
    ground_pc->clear();

    // filter those height are less than lpr.height+th_seeds_
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (p_sorted);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (lpr_height-th_seeds_, lpr_height+th_seeds_);
    pass.filter (*ground_pc);
    // update seeds points
}

void  MuxSeg::imuCallback_(const sensor_msgs::Imu::ConstPtr& msg){
    *imu_raw = *msg;
}

void MuxSeg::sensorCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, const std::string &topic){
    // {const std::lock_guard<std::mutex> lock(mutex_);
        if(topic == "camera_in_right"){
            if(get_cam_r_2_base_Matrix==false){get_cam_r_2_base_Matrix = get_trans_matrix(base_FrameId, right_camera_FrameId, cam_r_2_base_Matrix);}
            if(get_cam_r_2_base_Matrix==true){pcl_ros::transformPointCloud(cam_r_2_base_Matrix, *input_cloud, *camera_cloud_right_transformed);}
        }else if(topic == "camera_in_left"){
            if(get_cam_l_2_base_Matrix==false){get_cam_l_2_base_Matrix = get_trans_matrix(base_FrameId, left_camera_FrameId, cam_l_2_base_Matrix);}
            if(get_cam_l_2_base_Matrix==true){pcl_ros::transformPointCloud(cam_l_2_base_Matrix, *input_cloud, *camera_cloud_left_transformed);}
        }else if(topic == "range_in_right"){
            if(get_range_r_2_base_Matrix==false){get_range_r_2_base_Matrix = get_trans_matrix(base_FrameId, right_range_FrameId, range_r_2_base_Matrix);}
            if(get_range_r_2_base_Matrix==true){pcl_ros::transformPointCloud(range_r_2_base_Matrix, *input_cloud, *range_cloud_right_transformed);}
        }else if(topic == "range_in_left"){
            if(get_range_l_2_base_Matrix==false){get_range_l_2_base_Matrix = get_trans_matrix(base_FrameId, left_range_FrameId, range_l_2_base_Matrix);}
            if(get_range_l_2_base_Matrix==true){pcl_ros::transformPointCloud(range_l_2_base_Matrix, *input_cloud, *range_cloud_left_transformed);}
        }else if(topic == "lidar_in_right"){
            if(get_lidar_r_2_base_Matrix==false){get_lidar_r_2_base_Matrix = get_trans_matrix(base_FrameId, right_lidar_FrameId, lidar_r_2_base_Matrix);}
            if(get_lidar_r_2_base_Matrix==true){pcl_ros::transformPointCloud(lidar_r_2_base_Matrix, *input_cloud, *lidar_cloud_right_transformed);}
        }else if(topic == "lidar_in_left"){
            if(get_lidar_l_2_base_Matrix==false){get_lidar_l_2_base_Matrix = get_trans_matrix(base_FrameId, left_lidar_FrameId, lidar_l_2_base_Matrix);}
            if(get_lidar_l_2_base_Matrix==true){pcl_ros::transformPointCloud(lidar_l_2_base_Matrix, *input_cloud, *lidar_cloud_left_transformed);}
        }else{ROS_INFO("Unknown topic name");}
    // }
}

void MuxSeg::timerCallback_ (){
    // ROS_INFO("timer callback"); 
    ros::Time begin = ros::Time::now();

    // 1. Create global PC
    pcl::PCLPointCloud2::Ptr cloud_pcl (new pcl::PCLPointCloud2 ());

    // 2. Merge all other PCs
    pcl::PCLPointCloud2::Ptr temp_pcl (new pcl::PCLPointCloud2 ());
    if(get_cam_r_2_base_Matrix==true){
        pcl_conversions::toPCL(*camera_cloud_right_transformed, *temp_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*temp_pcl);
    }
    if(get_cam_l_2_base_Matrix==true){
        pcl_conversions::toPCL(*camera_cloud_left_transformed, *temp_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*temp_pcl);
    }
    if(get_range_r_2_base_Matrix==true){
        pcl_conversions::toPCL(*range_cloud_right_transformed, *temp_pcl);
        // pcl::removeNaNFromPointCloud(*range_cloud_right_pcl,*range_cloud_right_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*temp_pcl);
    }
    if(get_range_l_2_base_Matrix==true){
        pcl_conversions::toPCL(*range_cloud_left_transformed, *temp_pcl);
        // pcl::removeNaNFromPointCloud(*range_cloud_left_pcl,*range_cloud_left_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*temp_pcl);
    }
    if(get_lidar_r_2_base_Matrix==true){
        pcl_conversions::toPCL(*lidar_cloud_right_transformed, *temp_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*temp_pcl);
    }
    if(get_lidar_l_2_base_Matrix==true){
        pcl_conversions::toPCL(*lidar_cloud_left_transformed, *temp_pcl);
        pcl::PCLPointCloud2::concatenate(*cloud_pcl,*temp_pcl);
    }

    Affine3d rot_matrix;
    // 3.Convert comb pc to pcl::PointXYZ
    if(cloud_pcl->width > 0){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::fromPCLPointCloud2(*cloud_pcl, *cloud_raw);

        double q0_in, q1_in, q2_in, q3_in; 
        q0_in=imu_raw->orientation.w;
        q1_in=imu_raw->orientation.x;
        q2_in=imu_raw->orientation.y;
        q3_in=imu_raw->orientation.z;

        // 4.Transform pointcloud w.r.t IMU reading
        quaternionToMatrix(q0_in, q1_in, q2_in, q3_in, rot_matrix);
        pcl::transformPointCloud (*cloud_raw, *cloud_raw, rot_matrix);

        if(debug){
            sensor_msgs::PointCloud2::Ptr OA_msg (new sensor_msgs::PointCloud2 ());
            pcl::toROSMsg(*cloud, *OA_msg);
            OA_msg->header.stamp = ros::Time::now();
            OA_msg->header.frame_id = base_FrameId;
            OA_pub_.publish(*OA_msg);
        }

        // 5.Clip based on box threshold.
        pcl::CropBox<pcl::PointXYZ> box;
        box.setMin(Vector4f(-th_box_, -th_box_, -1.0,   1.0));
        box.setMax(Vector4f( th_box_,  th_box_, th_ceil_,   1.0));
        box.setInputCloud(cloud_raw);
        box.filter(*cloud_raw);

        box.setMin(Vector4f(-dead_zone_-0.1, -dead_zone_, -0.3,   1.0));
        box.setMax(Vector4f( dead_zone_+0.28,  dead_zone_,  0.3,   1.0));
        box.setInputCloud(cloud_raw);
        box.setNegative (true);
        box.filter(*cloud_raw);
        pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(*cloud_raw, *cloud);

        // 6.Apply voxel filter
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (float (0.2*map_unit_size_), float (0.2*map_unit_size_), float (2));
        sor.filter (*cloud);

        if(debug){
            sensor_msgs::PointCloud2::Ptr VGF_msg (new sensor_msgs::PointCloud2 ());
            pcl::toROSMsg(*cloud, *VGF_msg);
            VGF_msg->header.stamp = ros::Time::now();
            VGF_msg->header.frame_id = base_FrameId;
            VGF_pub_.publish(*VGF_msg);
        }

        // 7. Filter those points above camera
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-2.0*sensor_height_, 0.0);
        pass.filter (*cloud);

        if(debug){
            sensor_msgs::PointCloud2::Ptr BC_msg (new sensor_msgs::PointCloud2 ());
            pcl::toROSMsg(*cloud, *BC_msg);
            BC_msg->header.stamp = ros::Time::now();
            BC_msg->header.frame_id = base_FrameId;
            BC_pub_.publish(*BC_msg);
        }

        // 8.Apply radius removal filter
        if (in_radius_>0){
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            outrem.setInputCloud(cloud);
            outrem.setRadiusSearch(radius_search_);
            outrem.setMinNeighborsInRadius (in_radius_);
            //outrem.setKeepOrganized(true);
            outrem.filter (*cloud);  
        }

        if(debug){
            sensor_msgs::PointCloud2::Ptr ROR_msg (new sensor_msgs::PointCloud2 ());
            pcl::toROSMsg(*cloud, *ROR_msg);
            ROR_msg->header.stamp = ros::Time::now();
            ROR_msg->header.frame_id = base_FrameId;
            ROR_pub_.publish(*ROR_msg);
        }

        // 10.Check if camera got block
        bool block = false;
        if(block_thres_>0){
            block = checkBlock(cloud, block_thres_);
        }

        if(debug){
            std::cout<< "block:" <<std::endl;
            std::cout<< block <<std::endl;
        }

       if(block == false){
            // 11.Sort on Z-axis value
            sort(cloud->points.begin(),cloud->end(),point_cmp);

            // 12. Extract init ground seeds
            extract_initial_seeds_(cloud);
            pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(*ground_pc, *cloud);

            if(debug){
                sensor_msgs::PointCloud2::Ptr SEED_msg (new sensor_msgs::PointCloud2 ());
                pcl::toROSMsg(*cloud, *SEED_msg);
                SEED_msg->header.stamp = ros::Time::now();
                SEED_msg->header.frame_id = base_FrameId;
                SEED_pub_.publish(*SEED_msg);
            }

            // 10. Ground plane fitter mainloop
            for(int i=0;i<num_iter_;i++){
                estimate_plane_();
                // Clear memory
                ground_pc->clear();
                // Threshold filter
                if(i<num_iter_-1){
                    for(size_t r=0;r<cloud->points.size();r++){
                        double dist = double((*cloud)[r].z);
                        if(SVD_refinement==true && SVD_holdoff==false){
                            Vector3d point;
                            point << (*cloud)[r].x,(*cloud)[r].y,(*cloud)[r].z;
                            dist = point.dot(normal_);
                        }
                        if(dist<th_dist_d_){
                            ground_pc->points.push_back((*cloud)[r]);
                        }
                    } 
                }else{ // Reach last iteration
                    not_ground_pc->clear();
                    for(size_t r=0;r<cloud_raw->points.size();r++){
                        double dist = double((*cloud_raw)[r].z);
                        if(SVD_refinement==true && SVD_holdoff==false){
                            Vector3d point;
                            point << (*cloud_raw)[r].x,(*cloud_raw)[r].y,(*cloud_raw)[r].z;
                            dist = point.dot(normal_);
                        }
                        if(dist<(th_dist_d_) && dist>(th_dist_d_-(3*th_dist_))){
                            ground_pc->points.push_back((*cloud_raw)[r]);
                        }else if(dist<=(th_dist_d_-(3*th_dist_))){
                            if(detect_neg==true){
                                if(compress_z){(*cloud_raw)[r].z = 0.0;}
                                not_ground_pc->points.push_back((*cloud_raw)[r]);
                            }else{
                                // Naive re-scale correction
                                // double scale_c = th_dist_d_/(*cloud_raw)[r].y;
                                // (*cloud_raw)[r].x = scale_c*(*cloud_raw)[r].x;
                                // (*cloud_raw)[r].y = scale_c*(*cloud_raw)[r].y;
                                // (*cloud_raw)[r].z = scale_c*(*cloud_raw)[r].z;
                                ground_pc->points.push_back((*cloud_raw)[r]);
                            }
                        }else{
                            if(compress_z){(*cloud_raw)[r].z = 0.0;}
                            not_ground_pc->points.push_back((*cloud_raw)[r]);
                        }
                    }
                }
            }
            last_d_ = th_dist_d_;
        }else{
            ground_pc->clear();
            not_ground_pc->clear();
            for(size_t r=0;r<cloud_raw->points.size();r++){

                double dist = double((*cloud_raw)[r].z);
                if(dist<(last_d_) && dist>(last_d_-(3*th_dist_))){
                    ground_pc->points.push_back((*cloud_raw)[r]);
                }else if(dist<=(last_d_-(3*th_dist_))){
                    if(detect_neg==true){
                        if(compress_z){(*cloud_raw)[r].z = 0.0;}
                        not_ground_pc->points.push_back((*cloud_raw)[r]);
                    }else{
                        ground_pc->points.push_back((*cloud_raw)[r]);
                    }
                }else{
                    if(compress_z){(*cloud_raw)[r].z = 0.0;}
                    not_ground_pc->points.push_back((*cloud_raw)[r]);
                }
                
            }
        }

        if(timer){
            std::cout<<"walltime: "<< ros::Time::now()-begin<<std::endl;
        }

        ros::Time stamp = ros::Time::now();
        // publish ground points
        sensor_msgs::PointCloud2::Ptr ground_msg (new sensor_msgs::PointCloud2 ());
        pcl::toROSMsg(*ground_pc, *ground_msg);
        ground_msg->header.stamp = stamp;
        ground_msg->header.frame_id = base_FrameId;
        ground_points_pub_.publish(*ground_msg);
        //publish not ground points
        sensor_msgs::PointCloud2::Ptr groundless_msg (new sensor_msgs::PointCloud2 ());
        pcl::toROSMsg(*not_ground_pc, *groundless_msg);
        groundless_msg->header.stamp = stamp;
        groundless_msg->header.frame_id = base_FrameId;
        groundless_points_pub_.publish(*groundless_msg);
        //publish ground plane params
        pc_gps::gpParam gp_param;
        gp_param.header.stamp = stamp;
        if(SVD_refinement==true && SVD_holdoff==false){
            normal_=rot_matrix.inverse()*normal_;
        }
        for(int i=0; i<4; i++){
            if(i==3){
                gp_param.data[i] = d_;
            }else{
                gp_param.data[i] = normal_(i,0);
            }
        }
        gp_param_pub_.publish(gp_param);
    }

    if(debug){
        visualization_msgs::Marker normal_vector;
        normal_vector.header.stamp = ros::Time::now();
        normal_vector.header.frame_id = base_FrameId;
        normal_vector.scale.x = 0.1;
        normal_vector.color.b = 1.0;
        normal_vector.color.a = 1.0;
        normal_vector.action = visualization_msgs::Marker::ADD;
        normal_vector.pose.orientation.w = 1.0;
        geometry_msgs::Point p0;
        p0.x = p0.y = p0.z = 0;
        p0.z = d_;
        geometry_msgs::Point p1;
        p1.x = normal_(0,0);
        p1.y = normal_(1,0);
        p1.z = normal_(2,0);
        normal_vector.points.push_back(p0);
        normal_vector.points.push_back(p1);
        normal_vec_pub_.publish(normal_vector);

        visualization_msgs::Marker imu_vector;
        imu_vector.header.stamp = ros::Time::now();
        imu_vector.header.frame_id = base_FrameId;
        imu_vector.scale.x = 0.1;
        imu_vector.color.r = 1.0;
        imu_vector.color.a = 1.0;
        imu_vector.action = visualization_msgs::Marker::ADD;
        imu_vector.pose.orientation.w = 1.0;
        p0.x = p0.y = p0.z = 0;
        p0.z = d_;
        normal_ << 0.0,0.0,1.0;
        normal_=rot_matrix.inverse()*normal_;
        p1.x = normal_(0,0);
        p1.y = normal_(1,0);
        p1.z = normal_(2,0);
        imu_vector.points.push_back(p0);
        imu_vector.points.push_back(p1);
        imu_vec_pub_.publish(imu_vector);
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "MuxSeg");
    MuxSeg node;
    
    // ros::MultiThreadedSpinner spinner(5); // Use multi threads
    // spinner.spin();

    // ros::AsyncSpinner spinner(2); // Use multi threads
    // spinner.start();

    ros::spin();

    return 0;
 }
