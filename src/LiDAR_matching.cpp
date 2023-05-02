//
// Created by echo on 23-5-2.
//
#include <iostream>
#include "LiDAR_matching_lib/LiDAR_matching_lib.h"
LiDAR_matching_lib LM;
void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    LM.ImuQueue.push(*msg);
    LM.newIMU = true;
}
void PointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    LM.Point_raw = *msg;
    LM.newLiDAR = true;
}
void InitPoseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    LM.T_map.translation()<<0,0,0;
    LM.InitPose = *msg;
}
int main(int argc, char** argv){
    bool debug = true;
    ros::init(argc, argv, "LiDAR_matching");
    ros::NodeHandle nh("~");
    LM.LoadMap("/home/echo/bag/7720_Lidar/map/output.pcd");
    ROS_WARN("Map Finish");
    ros::Subscriber sub1 = nh.subscribe("/imu/data_raw", 1000, ImuCallback);
    ros::Subscriber sub2 = nh.subscribe("/velodyne_points", 1000, PointCallback);
    ros::Subscriber sub3 = nh.subscribe("/init_position", 1000, InitPoseCallback);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1000);
    ros::Publisher LiDAR_pub = nh.advertise<sensor_msgs::PointCloud2>("/LiDAR_Distortion", 1000);
    ros::Publisher Local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1000);
    if(debug){
        LM.T_map.setIdentity();
        Eigen::Quaterniond q;
        q.x() = -0.0383515320718; q.y() =0.0165229793638; q.z() = 0.543365836143; q.w() = 0.838456749916;
        LM.T_map.rotate(q.matrix()) ;
        LM.T_map.translation()<<0,0,0;
        LM.InitPoseBool = true;
    }

    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        LM.process();
        //map_pub.publish(LM.mls_map);
        LiDAR_pub.publish(LM.LiDAR_Distort);
        Local_map_pub.publish(LM.LocalMapPC2);
        r.sleep();
    }
}
