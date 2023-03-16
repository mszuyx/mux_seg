// Import ROS lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
// Import PCL lib
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
// Import Eigen lib
#include <Eigen/Dense> 
// Import message_filters lib
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Custom msg type
#include <pc_gps/gpParam.h>
// Import tf
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <mutex> 

using namespace message_filters;
using namespace Eigen;

// Declare helper functions
bool point_cmp(const pcl::PointXYZ a, const pcl::PointXYZ b){return a.y>b.y;}

// Declare pc variables
pcl::PointCloud<pcl::PointXYZ>::Ptr seeds_pc(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pc(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr not_ground_pc(new pcl::PointCloud<pcl::PointXYZ>());

// Shared pointcloud variables
sensor_msgs::PointCloud2::Ptr range_cloud_right_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr range_cloud_left_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr lidar_cloud_right_transformed(new sensor_msgs::PointCloud2 ());
sensor_msgs::PointCloud2::Ptr lidar_cloud_left_transformed(new sensor_msgs::PointCloud2 ());

std::mutex mutex_;

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

    // Declare ROS params
    double sensor_height_;
    int num_iter_;
    int num_lpr_;
    double th_seeds_;
    double th_dist_;
    double th_ceil_;
    double th_box_;
    double map_unit_size_;
    // std::string base_FrameId;
    std::string right_camera_FrameId;
    std::string left_camera_FrameId;
    std::string right_range_FrameId;
    std::string left_range_FrameId;
    std::string right_lidar_FrameId;
    std::string left_lidar_FrameId;
    double radius_search_;
    int in_radius_;
    double std_th_;
    int mean_k_;
    double alpha;
    bool SVD_refinement;
    bool dense;
    
    // Sync settings
    message_filters::Subscriber<sensor_msgs::PointCloud2> points1_node_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> points2_node_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_node_sub_;
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::Imu> RSSyncPolicy;
    typedef Synchronizer<RSSyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    ros::Subscriber range1_node_sub_;
    ros::Subscriber range2_node_sub_;
    ros::Subscriber lidar1_node_sub_;
    ros::Subscriber lidar2_node_sub_;


    // Declare functions
    void quaternionMultiplication(double p0, double p1, double p2, double p3, double q0, double q1, double q2, double q3, double& r0, double& r1, double& r2, double& r3);
    void rotateVectorByQuaternion(double x, double y, double z, double q0, double q1, double q2, double q3, double& vx, double& vy, double& vz);
    void quaternionToMatrix(double q0, double q1, double q2, double q3,  Affine3d& transform);
    void imuNormal_update(double p0, double p1, double p2, double p3);
    void findMean(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, Vector3d& pc_mean);
    void estimate_plane_(void);
    void extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZ>::Ptr& p_sorted);
    void rs_pc_callback_ (const sensor_msgs::PointCloud2::ConstPtr& input_cloud1, const sensor_msgs::PointCloud2::ConstPtr& input_cloud2, const sensor_msgs::Imu::ConstPtr& imu_msg);
    void merge_point32_vector(const std::vector<geometry_msgs::Point32>& vin, std::vector<geometry_msgs::Point32>& vout);

    void secondaryCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, const std::string &topic);
    bool get_trans_matrix(const std::string &parentFrame, const std::string &childFrame, Matrix4f& tfMatrix);

    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0, here normal:=[a,b,c], d=d, th_dist_d_ = threshold_dist - d 
    double d_;
    Vector3d normal_imu;
    Vector3d normal_;
    double th_dist_d_;
    double last_lpr;

    // Matrix4f cam_r_2_cam_r_Matrix;
    // bool get_cam_r_2_cam_r_Matrix = false;
    Matrix4f cam_l_2_cam_r_Matrix;
    bool get_cam_l_2_cam_r_Matrix = false;
    Matrix4f range_r_2_cam_r_Matrix;
    bool get_range_r_2_cam_r_Matrix = false;
    Matrix4f range_l_2_cam_r_Matrix;
    bool get_range_l_2_cam_r_Matrix = false;
    Matrix4f lidar_r_2_cam_r_Matrix;
    bool get_lidar_r_2_cam_r_Matrix = false;
    Matrix4f lidar_l_2_cam_r_Matrix;
    bool get_lidar_l_2_cam_r_Matrix= false;
};

MuxSeg::MuxSeg():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing Ground Plane Segmentation Node...");

    node_handle_.param("sensor_height", sensor_height_, 1.0);
    ROS_INFO("Sensor Height: %f", sensor_height_);

    node_handle_.param("num_iter", num_iter_, 3);
    ROS_INFO("Num of Iteration: %d", num_iter_);

    node_handle_.param("num_lpr", num_lpr_, 400);
    ROS_INFO("Num of LPR: %d", num_lpr_);

    node_handle_.param("th_seeds", th_seeds_, 0.5);
    ROS_INFO("Seeds Threshold: %f", th_seeds_);

    node_handle_.param("th_dist", th_dist_, 0.05);
    ROS_INFO("Distance Threshold: %f", th_dist_);

    node_handle_.param("th_ceil_", th_ceil_, 1.0);
    ROS_INFO("Ceiling Threshold: %f", th_ceil_);

    node_handle_.param("th_box_", th_box_, 7.0);
    ROS_INFO("Box Threshold: %f", th_box_);

    node_handle_.param("map_unit_size_", map_unit_size_, 0.15); 
    ROS_INFO("map_unit_size_: %f", map_unit_size_);

    node_handle_.param("radius_search_", radius_search_, 0.15);
    ROS_INFO("radius_search_: %f", radius_search_);

    node_handle_.param("in_radius_", in_radius_, -25);
    ROS_INFO("in_radius_: %d", in_radius_);

    node_handle_.param("std_th_", std_th_, 0.12);
    ROS_INFO("std_th_: %f", std_th_);

    node_handle_.param("mean_k_", mean_k_, -15);
    ROS_INFO("mean_k_: %d", mean_k_);

    node_handle_.param("alpha", alpha, 0.1);
    ROS_INFO("moving average factor alpha: %f", alpha);

    node_handle_.param("SVD_refinement", SVD_refinement, false);
    ROS_INFO("Do SVD refinement?: %s", SVD_refinement ? "true" : "false");

    node_handle_.param("dense", dense, false);
    ROS_INFO("return dense point cloud?: %s", dense ? "true" : "false");

    // node_handle_.param<std::string>("base_frame_id", base_FrameId, "/base_link");
    // ROS_INFO("base frame_id: %s", base_FrameId.c_str());

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

    // Subscribe to realsense topic
    points1_node_sub_.subscribe(node_handle_, "/cloud_in_1", 1); //5
    points2_node_sub_.subscribe(node_handle_, "/cloud_in_2", 1); //5
    imu_node_sub_.subscribe(node_handle_, "/imu/data", 1);  //100

    // ApproximateTime takes a queue size as its constructor argument, hence RSSyncPolicy(xx)
    sync.reset(new Sync(RSSyncPolicy(10), points1_node_sub_, points2_node_sub_, imu_node_sub_));   
    sync->registerCallback(boost::bind(&MuxSeg::rs_pc_callback_, this, _1, _2, _3));

    // Subscribe to secondary sensors
    range1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/range_in_right", 1, boost::bind(&MuxSeg::secondaryCallback_, this, _1, "range_in_right")); //5
    range2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/range_in_left", 1, boost::bind(&MuxSeg::secondaryCallback_, this, _1, "range_in_left")); //5
    lidar1_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_in_right", 1, boost::bind(&MuxSeg::secondaryCallback_, this, _1, "lidar_in_right")); //5
    lidar2_node_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/lidar_in_left", 1, boost::bind(&MuxSeg::secondaryCallback_, this, _1, "lidar_in_left")); //5

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

    normal_imu << 0.0,1.0,0.0;
    normal_ << 0.0,1.0,0.0;
    last_lpr = sensor_height_;
}

void MuxSeg::quaternionMultiplication(double p0, double p1, double p2, double p3,
                                              double q0, double q1, double q2, double q3,
                                              double& r0, double& r1, double& r2, double& r3){
    // r = p q
    r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
    r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
    r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1;
    r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0;
}

void MuxSeg::rotateVectorByQuaternion(double x, double y, double z, 
                                              double q0, double q1, double q2, double q3, 
                                              double& vx, double& vy, double& vz){ 
    vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z;
    vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z;
    vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z;
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
    transform = AngleAxisd(pitch, Vector3d::UnitZ()) * AngleAxisd(roll+1.5708, Vector3d::UnitX()); 
}

void MuxSeg::imuNormal_update(double q0, double q1, double q2, double q3){
    // Rotate quaternion into proper frame:
    double q0_, q1_, q2_, q3_; 
    double q0_tf=0, q1_tf=0, q2_tf=0, q3_tf=1; 
    quaternionMultiplication(q0, q1, q2, q3,
                            q0_tf, q1_tf, q2_tf, q3_tf,
                            q0_, q1_, q2_, q3_);
    double ux=0, uy=0, uz=1;
    double lx, ly, lz;
    rotateVectorByQuaternion(ux, uy, uz,
                            q0_, -q1_, -q2_, -q3_, 
                            lx, ly, lz);
                                            
    normal_imu << lx,ly,-lz;
}

void MuxSeg::findMean(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, Vector3d& pc_mean){
    double x_sum = 0;
    double y_sum = 0;
    double z_sum = 0;
    int cnt = 0;
    for(size_t i=0;i<pc->points.size();i++){
        x_sum += double(pc->points[i].x);
        y_sum += double(pc->points[i].y);
        z_sum += double(pc->points[i].z);
        cnt++;
    }
    x_sum = cnt!=0?x_sum/cnt:0;
    y_sum = cnt!=0?y_sum/cnt:0;
    z_sum = cnt!=0?z_sum/cnt:0;
    pc_mean << x_sum,y_sum,z_sum;
}

void MuxSeg::estimate_plane_(void){
    if(SVD_refinement==true){
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
        d_ = -(normal_.transpose()*seeds_mean)(0,0);
    }else{
        Vector3d seeds_mean;
        findMean(ground_pc, seeds_mean);
        d_ = -seeds_mean[1];
    }
    // set distance threhold to `th_dist - d`
    th_dist_d_ = -(th_dist_ + d_);
    // update the equation parameters
}

void MuxSeg::extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZ>::Ptr& p_sorted){
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for(size_t i=0;i<p_sorted->points.size() && cnt<num_lpr_;i++){
        sum += double(p_sorted->points[i].y);
        cnt++;
    }
    double lpr_height = cnt!=0?sum/cnt:0;// in case divide by 0
    lpr_height = (alpha*last_lpr) + ((1-alpha)*lpr_height);

    seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for(size_t i=0;i<p_sorted->points.size();i++){
        if(double(p_sorted->points[i].y) > lpr_height + th_seeds_){
            seeds_pc->points.push_back(p_sorted->points[i]);
        }
    }
    last_lpr = lpr_height;
    // update seeds points
}

void MuxSeg::merge_point32_vector(const std::vector<geometry_msgs::Point32>& vin, std::vector<geometry_msgs::Point32>& vout){
    vout.reserve(vout.size() + vin.size());
    vout.insert(vout.end(), vin.begin(), vin.end());
}

bool MuxSeg::get_trans_matrix(const std::string &parentFrame, const std::string &childFrame, Matrix4f& tfMatrix){
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

void MuxSeg::secondaryCallback_(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, const std::string &topic){
    {const std::lock_guard<std::mutex> lock(mutex_);
        // ROS_INFO("callback"); 
        if(topic == "range_in_right"){
            if(get_range_r_2_cam_r_Matrix==false){
                get_range_r_2_cam_r_Matrix = get_trans_matrix(right_camera_FrameId, right_range_FrameId, range_r_2_cam_r_Matrix);
                std::cout<< right_range_FrameId <<std::endl;
            }else{pcl_ros::transformPointCloud(range_r_2_cam_r_Matrix, *input_cloud, *range_cloud_right_transformed);}
            
        }else if(topic == "range_in_left"){
            if(get_range_l_2_cam_r_Matrix==false){
                get_range_l_2_cam_r_Matrix = get_trans_matrix(right_camera_FrameId, left_range_FrameId, range_l_2_cam_r_Matrix);
                std::cout<< left_range_FrameId <<std::endl;
            }else{pcl_ros::transformPointCloud(range_l_2_cam_r_Matrix, *input_cloud, *range_cloud_left_transformed);}

        }else if(topic == "lidar_in_right"){
            if(get_lidar_r_2_cam_r_Matrix==false){
                get_lidar_r_2_cam_r_Matrix = get_trans_matrix(right_camera_FrameId, right_lidar_FrameId, lidar_r_2_cam_r_Matrix);
                std::cout<< right_lidar_FrameId <<std::endl;
            }else{pcl_ros::transformPointCloud(lidar_r_2_cam_r_Matrix, *input_cloud, *lidar_cloud_right_transformed);}
            
        }else if(topic == "lidar_in_left"){
            if(get_lidar_l_2_cam_r_Matrix==false){
                get_lidar_l_2_cam_r_Matrix = get_trans_matrix(right_camera_FrameId, left_lidar_FrameId, lidar_l_2_cam_r_Matrix);
                std::cout<< left_lidar_FrameId <<std::endl;
            }else{pcl_ros::transformPointCloud(lidar_l_2_cam_r_Matrix, *input_cloud, *lidar_cloud_left_transformed);}
            
        }else{
            ROS_INFO("Unknown topic name"); 
        }
    }
}


void MuxSeg::rs_pc_callback_ (const sensor_msgs::PointCloud2::ConstPtr& input_cloud1, const sensor_msgs::PointCloud2::ConstPtr& input_cloud2, const sensor_msgs::Imu::ConstPtr& imu_msg){
    // ROS_INFO("callback"); 

    double q0_in, q1_in, q2_in, q3_in; 
    q0_in=imu_msg->orientation.w;
    q1_in=imu_msg->orientation.x;
    q2_in=imu_msg->orientation.y;
    q3_in=imu_msg->orientation.z;
     
    // Update ground plane normal vector based on the assumption that the normal vector is parallel to the gravity vector                         
    imuNormal_update(q0_in, q1_in, q2_in, q3_in);

    // 1. Merge left camera PCs
    // if(get_cam_r_2_cam_r_Matrix==false){
    //     get_cam_r_2_cam_r_Matrix = get_trans_matrix(base_FrameId, right_camera_FrameId, cam_r_2_cam_r_Matrix);
    //     std::cout<< right_camera_FrameId <<std::endl;
    // }
    // sensor_msgs::PointCloud2::Ptr input_cloud1_transformed (new sensor_msgs::PointCloud2 ());
    // pcl_ros::transformPointCloud(cam_r_2_cam_r_Matrix, *input_cloud1, *input_cloud1_transformed);
    pcl::PCLPointCloud2::Ptr input_cloud1_pcl (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*input_cloud1, *input_cloud1_pcl);
    
    if(get_cam_l_2_cam_r_Matrix==false){
        get_cam_l_2_cam_r_Matrix = get_trans_matrix(right_camera_FrameId, left_camera_FrameId, cam_l_2_cam_r_Matrix);
        std::cout<< left_camera_FrameId <<std::endl;
    }
    sensor_msgs::PointCloud2::Ptr input_cloud2_transformed (new sensor_msgs::PointCloud2 ());
    pcl_ros::transformPointCloud(cam_l_2_cam_r_Matrix, *input_cloud2, *input_cloud2_transformed);
    pcl::PCLPointCloud2::Ptr input_cloud2_pcl (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*input_cloud2_transformed, *input_cloud2_pcl);
    pcl::PCLPointCloud2::concatenate(*input_cloud1_pcl,*input_cloud2_pcl);
    

    //  Apply voxel filter
    if(dense==false){
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (input_cloud1_pcl);
        sor.setLeafSize (float (0.3*map_unit_size_),float (map_unit_size_),float (0.3*map_unit_size_));
        sor.filter (*input_cloud1_pcl);
    }

    if(get_range_r_2_cam_r_Matrix==true){
        pcl::PCLPointCloud2::Ptr range_cloud_right_pcl (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*range_cloud_right_transformed, *range_cloud_right_pcl);
        // pcl::removeNaNFromPointCloud(*range_cloud_right_pcl,*range_cloud_right_pcl);
        pcl::PCLPointCloud2::concatenate(*input_cloud1_pcl,*range_cloud_right_pcl);
    }
    if(get_range_l_2_cam_r_Matrix==true){
        pcl::PCLPointCloud2::Ptr range_cloud_left_pcl (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*range_cloud_left_transformed, *range_cloud_left_pcl);
        // pcl::removeNaNFromPointCloud(*range_cloud_left_pcl,*range_cloud_left_pcl);
        pcl::PCLPointCloud2::concatenate(*input_cloud1_pcl,*range_cloud_left_pcl);
    }
    if(get_lidar_r_2_cam_r_Matrix==true){
        pcl::PCLPointCloud2::Ptr lidar_cloud_right_pcl (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*lidar_cloud_right_transformed, *lidar_cloud_right_pcl);
        pcl::PCLPointCloud2::concatenate(*input_cloud1_pcl,*lidar_cloud_right_pcl);
    }
    if(get_lidar_l_2_cam_r_Matrix==true){
        pcl::PCLPointCloud2::Ptr lidar_cloud_left_pcl (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*lidar_cloud_left_transformed, *lidar_cloud_left_pcl);
        pcl::PCLPointCloud2::concatenate(*input_cloud1_pcl,*lidar_cloud_left_pcl);
    }


    // 0.Convert pc to pcl::PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(*input_cloud1_pcl, *cloud_raw);

    // 2.Transform pointcloud w.r.t IMU reading
    Affine3d transform;
    quaternionToMatrix(q0_in, q1_in, q2_in, q3_in, transform);
    pcl::transformPointCloud (*cloud_raw, *cloud_raw, transform);
    pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(*cloud_raw, *cloud);

    // 3.Apply voxel filter
    if(dense==true){
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (float (0.3*map_unit_size_),float (map_unit_size_),float (0.3*map_unit_size_));
        sor.filter (*cloud);
    }

    // 4.Clip based on box threshold.
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Vector4f(-th_box_, -1.5f, 0.0f, 1.0f));
    boxFilter.setMax(Vector4f(th_box_, 2.0*sensor_height_, th_box_, 1.0f));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cloud);

    // 5.Apply radius removal filter
    if (in_radius_>0){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(radius_search_);
        outrem.setMinNeighborsInRadius (in_radius_);
        //outrem.setKeepOrganized(true);
        outrem.filter (*cloud);  
    }

    // 6.Apply statistical outlier removal
    if (mean_k_>0){
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (mean_k_);
        sor.setStddevMulThresh (std_th_);
        sor.filter (*cloud);
    }

    // 7.Sort on Y-axis value
    // sort(cloud->points.begin(),cloud->end(),point_cmp);

    // 8. Extract init ground seeds
    // extract_initial_seeds_(cloud);
    // ground_pc = seeds_pc;

    // 9. Ground plane fitter mainloop
    for(int i=0;i<num_iter_;i++){
        if(i>0){estimate_plane_();}
        // Clear memory
        ground_pc->clear();
        // Threshold filter
        if(i<num_iter_-1){
            for(size_t r=0;r<cloud->points.size();r++){
                double dist = double((*cloud)[r].y);
                if(SVD_refinement==true && i>0){
                    Vector3d point;
                    point << (*cloud)[r].x,(*cloud)[r].y,(*cloud)[r].z;
                    dist = point.dot(normal_);
                }
                if(dist>th_dist_d_){ // Maybe add a "<" constraint?//
                    ground_pc->points.push_back((*cloud)[r]);
                }
            } 
        }else{ // Reach last iteration
            not_ground_pc->clear();
            for(size_t r=0;r<cloud_raw->points.size();r++){
                double dist = double((*cloud_raw)[r].y);
                double adj_th_ = 0.02*(*cloud_raw)[r].z;
            	// double adj_th_ = 0.25*(pow(0.1*(*cloud_raw)[r].z,2));
                if(SVD_refinement==true){
                    Vector3d point;
                    point << (*cloud_raw)[r].x,(*cloud_raw)[r].y,(*cloud_raw)[r].z;
                    dist = point.dot(normal_); 
                }
                if(dist>(th_dist_d_-adj_th_) && dist<(th_dist_d_+adj_th_+(3*th_dist_))){
                    ground_pc->points.push_back((*cloud_raw)[r]);
                }else if(dist<-th_ceil_){
                    continue;
                }//else if(dist<=0.01){
                 //   continue;
                //}
                else{
                    not_ground_pc->points.push_back((*cloud_raw)[r]);
                }
            }
        }
    }
    // std::cout<< "normal_:" <<std::endl;
    // std::cout<< normal_ <<std::endl;
    // std::cout<< "normal_imu:" <<std::endl;
    // std::cout<< normal_imu <<std::endl;

    // publish ground points
    sensor_msgs::PointCloud2::Ptr ground_msg (new sensor_msgs::PointCloud2 ());
    pcl::toROSMsg(*ground_pc, *ground_msg);
    ground_msg->header.stamp = input_cloud1->header.stamp;
    ground_msg->header.frame_id = input_cloud1->header.frame_id;
    ground_points_pub_.publish(*ground_msg);
    //publish not ground points
    sensor_msgs::PointCloud2::Ptr groundless_msg (new sensor_msgs::PointCloud2 ());
    pcl::toROSMsg(*not_ground_pc, *groundless_msg);
    groundless_msg->header.stamp = input_cloud1->header.stamp;
    groundless_msg->header.frame_id = input_cloud1->header.frame_id;
    groundless_points_pub_.publish(*groundless_msg);
    //publish ground plane params
    pc_gps::gpParam gp_param;
    gp_param.header.stamp = input_cloud1->header.stamp;
    for(int i=0; i<4; i++){
        if(i==3){
            gp_param.data[i] = d_;
        }else{
            gp_param.data[i] = normal_imu(i,0);
        }
    }
    gp_param_pub_.publish(gp_param);

}

int main (int argc, char** argv) {
    ros::init(argc, argv, "MuxSeg");
    MuxSeg node;
    ros::MultiThreadedSpinner spinner(5); // Use multi threads
    spinner.spin();
    // ros::spin();
    return 0;
 }
