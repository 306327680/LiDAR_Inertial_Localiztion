//
// Created by echo on 23-5-2.
//

#ifndef LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H
#define LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H

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
#include "pointType/pointTypes.h"
#include <registration/registration.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Path.h>
class LiDAR_matching_lib {
public:
    LiDAR_matching_lib(){};
    void LoadMap(std::string map_path);
    void LoadNormalMap(std::string map_path);
    void process();
    //1.Input
    sensor_msgs::PointCloud2 Point_raw;

    bool InitPoseBool = false;
    bool InitPoseCome = false;
    bool newIMU = false;
    bool newLiDAR = false;
    std::queue<sensor_msgs::Imu> ImuQueue;
    std::vector<double> IMU_Time;
    std::vector<Eigen::Quaterniond> IMU_q;
    //2.Output
    pcl::PointCloud<pcl::PointNormal> LocalMap;
    sensor_msgs::PointCloud2 LocalMapPC2;
    sensor_msgs::PointCloud2 LiDAR_Map;
    sensor_msgs::PointCloud2 LiDAR_Map_V;
    sensor_msgs::PointCloud2 LiDAR_Deskew;
    nav_msgs::Path IMU_path;
    nav_msgs::Odometry  LiDAR_map;
    nav_msgs::Odometry  LiDAR_map_last;
    sensor_msgs::PointCloud2 mls_map;
    //3.Variables
    pcl::PointCloud<pcl::PointNormal> mls_points;
    Eigen::Affine3d T_map;
    double *DebugTime = new double[5];

private:
    void InputDownSample();
    void ImuDistortion(double first_point_time,double last_point_time);
    void setInitParam();
    void genLocalMap();
    void registrion(pcl::PointCloud<pcl::PointXYZI> source,
                    pcl::PointCloud<pcl::PointNormal> target);
    void handleMessage();
    void findRotation(double pointTime, Eigen::Quaterniond &Q);

    bool imuReady = false;
   //1. params
    float IMU_period_time = 0.005;
    Eigen::Vector3d gyro_last;
    Eigen::Quaterniond IMU_predict_q;
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree;
    pcl::PointCloud<VLPPoint> vlp_pcd;
    pcl::PointCloud<pcl::PointXYZI> vlp_ds_pcd;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    std::vector<float> distances; // 存储近邻点对应距离的平方
    double FrameTime =0;
    double LastFrameTime =0;
    registration icp;
    pcl::PointCloud<pcl::PointXYZI> Transfer_local_point;
};


#endif //LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H
