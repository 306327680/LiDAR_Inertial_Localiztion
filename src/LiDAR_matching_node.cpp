//
// Created by echo on 23-5-2.
//
#include <iostream>
#include "LiDAR_matching_lib/LiDAR_matching_lib.h"
LiDAR_matching_lib LM;
void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    LM.ImuQueue.push_back(*msg);
    LM.newIMU = true;
}
void PointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    LM.Point_raw_queue.push(*msg);
    LM.newLiDAR = true;
}
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    LM.T_map.setIdentity();
    Eigen::Quaterniond q;
    q.x() = msg->pose.pose.orientation.x; q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z; q.w() =msg->pose.pose.orientation.w;
    LM.T_map.rotate(q.matrix()) ;
    LM.T_map.translation()<<msg->pose.pose.position.x,msg->pose.pose.position.y,0;
    LM.InitPoseBool = true;
    LM.IMU_q.push_back( q  );//todo clear
    LM.IMU_Time.push_back(msg->header.stamp.toSec());
}
int main(int argc, char** argv){
    bool debug = true;
    ros::init(argc, argv, "LiDAR_matching");
    std::cout<<argv[1]<<std::endl;
    LM.time_offset = std::atof(argv[1]);
    ros::NodeHandle nh("~");
    LM.LoadNormalMap("/home/echo/bag/7720_Lidar/map/map_smooth_ds.pcd");
    ROS_WARN("Map Finish");
    ros::Subscriber sub1 = nh.subscribe("/imu/data_raw", 50, ImuCallback);
    ros::Subscriber sub2 = nh.subscribe("/velodyne_points", 5, PointCallback);
    ros::Subscriber sub3 = nh.subscribe("/initialpose", 1, InitPoseCallback);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
    ros::Publisher LiDAR_pub = nh.advertise<sensor_msgs::PointCloud2>("/LiDAR_map_Distortion", 1);
    ros::Publisher Local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/LiDAR_map_position", 1);
    ros::Publisher Imu_LiDAR_pub = nh.advertise<nav_msgs::Odometry>("/LiDAR_at_IMU_time", 1);
    ros::Publisher Time_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>    ("/Time_cost", 1);
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    if(debug){
        LM.T_map.setIdentity();
        LM.extrinsicRot.setZero();//(1, 0, 0, 0, -1, 0, 0, 0, -1);
        LM.extrinsicRot(0,0) = 1;
        LM.extrinsicRot(1,1) = -1;
        LM.extrinsicRot(2,2) = -1;
        Eigen::Quaterniond q;
        //start
        q.x() = -0.0383515320718; q.y() =0.0165229793638; q.z() = 0.543365836143; q.w() = 0.838456749916;
        LM.T_map.rotate(q.matrix()) ;
        LM.T_map.translation()<<0,0,0;

//        q.x() = 0.0108444616199; q.y() = -0.12032815069; q.z() = -0.151517733932; q.w() =0.981043279171;
//        LM.T_map.rotate(q.matrix()) ;
//        LM.T_map.translation()<<271.907041259,427.126210513,0;

        LM.IMU_q.push_back( q );//todo clear
        LM.IMU_Time.push_back(0);
        LM.InitPoseBool = true;
    }

    ros::Rate r(1000);
    while(ros::ok()){
        if(LM.newLiDAR){
            LM.process();
            map_pub.publish(LM.mls_map);
            LiDAR_pub.publish(LM.LiDAR_Map);
            Local_map_pub.publish(LM.LocalMapPC2);
            odom_pub.publish(LM.LiDAR_map);
            Imu_LiDAR_pub.publish(LM.LiDAR_at_IMU_Time);
            Time_pub.publish(LM.Time_used);
            transform.setOrigin(tf::Vector3(LM.LiDAR_map.pose.pose.position.x,LM.LiDAR_map.pose.pose.position.y,LM.LiDAR_map.pose.pose.position.z));
            tf::Quaternion q(LM.LiDAR_map.pose.pose.orientation.x,LM.LiDAR_map.pose.pose.orientation.y,
                             LM.LiDAR_map.pose.pose.orientation.z,LM.LiDAR_map.pose.pose.orientation.w);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, LM.LiDAR_map.header.stamp, "map", "velodyne"));
            LM.newLiDAR = false;
        }
        ros::spinOnce();
        r.sleep();
    }
}
