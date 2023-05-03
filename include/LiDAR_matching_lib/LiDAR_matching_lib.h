//
// Created by echo on 23-5-2.
//

#ifndef LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H
#define LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
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
    //2.Output
    pcl::PointCloud<pcl::PointNormal> LocalMap;
    sensor_msgs::PointCloud2 LocalMapPC2;
    sensor_msgs::PointCloud2 LiDAR_Distort;
    sensor_msgs::PointCloud2 mls_map;
    //3.Variables
    pcl::PointCloud<pcl::PointNormal> mls_points;
    Eigen::Affine3d T_map;

private:
    void ImuDistortion(double first_point_time,double last_point_time);
    void registrion(pcl::PointCloud<VLPPoint> source,
                    pcl::PointCloud<pcl::PointNormal> target);
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree;
    pcl::PointCloud<VLPPoint> vlp_pcd;


    std::vector<float> distances; // 存储近邻点对应距离的平方
    double FrameTime =0;
    registration icp;
    pcl::PointCloud<pcl::PointXYZI> Transfer_local_point;
};


#endif //LIDAR_INERTIA_LOCALIZTION_LIDAR_MATCHING_LIB_H
