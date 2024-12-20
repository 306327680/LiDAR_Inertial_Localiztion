//
// Created by echo on 23-5-2.
//

#ifndef LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H
#define LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H
#include <time.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pointType/pointTypes.h"
#include <registration/registration.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <nav_msgs/Path.h>
#include <diagnostic_msgs/DiagnosticArray.h>
class LiDAR_matching_lib {
public:
    LiDAR_matching_lib(){};
    void LoadMap(std::string map_path);
    void LoadNormalMap(std::string map_path);
    void process();
    //1.Input
//    sensor_msgs::PointCloud2 Point_raw;
    sensor_msgs::PointCloud2 Point_raw;
    nav_msgs::Odometry IMU_pose;
    bool InitPoseBool = false;
    bool InitPoseCome = false;
    bool newIMU = false;
    bool newLiDAR = false;
    bool newPose = false;
    int PoseCnt = 0;
    std::vector<sensor_msgs::Imu> ImuQueue;
    std::vector<double> IMU_Time;
    std::vector<Eigen::Quaterniond> IMU_q;
    std::vector<Eigen::Vector3d> IMU_p;
    std::vector<Eigen::Vector3d> IMU_v;
    Eigen::Vector3d IMU_p_latest;
    Eigen::Quaterniond IMU_q_latest;
    Eigen::Affine3d IMU_pose_latest;
    //2.Output
    pcl::PointCloud<pcl::PointNormal> LocalMap;
    sensor_msgs::PointCloud2 LocalMapPC2;
    sensor_msgs::PointCloud2 LiDAR_Map;
    nav_msgs::Path IMU_predict_path;
    nav_msgs::Path map_path;
    nav_msgs::Path imu_constraint_path;
    nav_msgs::Path gt_path;
    diagnostic_msgs::DiagnosticStatus Time_used;

    nav_msgs::Odometry  LiDAR_map;
    nav_msgs::Odometry  LiDAR_at_IMU_Time;
    nav_msgs::Odometry  ICP_adjust_error;
    sensor_msgs::PointCloud2 mls_map;
    //3.Variables
    pcl::PointCloud<pcl::PointNormal> mls_points;
    Eigen::Affine3d T_IMU_preintergration;
    Eigen::Affine3d T_map;
    Eigen::Affine3d T_IMU_predict;
    Eigen::Affine3d T_map_last;

    //4. extric parameter
    Eigen::Matrix3d extrinsicRot;
    double time_offset = 0;
    bool Usemid360 = false;
private:
    void AccumulateImu();
    void InputDownSample();
    void ImuDistortion(double first_point_time,double last_point_time);

    void genLocalMap();
    void registrion(pcl::PointCloud<pcl::PointXYZI> source,
                    pcl::PointCloud<pcl::PointNormal> target);
    void handleMessage();
    void findRotation(double pointTime, Eigen::Quaterniond &Q);
    void findTrans(double pointTime, Eigen::Vector3d &T);
    void saveProcessTime(std::string name, double value);
    bool imuReady = false;
   //1. params
    float IMU_period_time = 0.0025;
    Eigen::Vector3d gyro_last;
    Eigen::Quaterniond IMU_predict_q;
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    pcl::PointCloud<VLPPoint> vlp_pcd;
    pcl::PointCloud<pcl::PointXYZI> vlp_ds_pcd;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    pcl::UniformSampling<pcl::PointXYZI> US;
    pcl::RandomSample<pcl::PointXYZI> RS;
    pcl::RandomSample<pcl::PointNormal> RS2;
    std::vector<int> indices_last;
    std::vector<float> distances; // 存储近邻点对应距离的平方
    double FrameTime =0;
    double LastFrameTime =0;
    registration icp;
    pcl::PointCloud<pcl::PointXYZI> Transfer_local_point;
};


#endif //LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H
