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
    LM.Point_raw = *msg;
    LM.newLiDAR = true;
}
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    LM.T_map.setIdentity();
    LM.T_map_last.setIdentity();
    Eigen::Quaterniond q;
    q.x() = msg->pose.pose.orientation.x; q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z; q.w() =msg->pose.pose.orientation.w;
    LM.T_map.rotate(q.matrix()) ;
    LM.T_map.translation()<<msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z;
    LM.IMU_p.push_back(Eigen::Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    LM.InitPoseBool = true;
    LM.IMU_q.push_back( q  );//todo clear
    LM.IMU_Time.push_back(msg->header.stamp.toSec());
}
void ImuPoseCallback(const nav_msgs::Odometry ::ConstPtr& msg){
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Eigen::Affine3d T;
    LM.IMU_pose = *msg;
    q.x() = LM.IMU_pose.pose.pose.orientation.x;
    q.y() = LM.IMU_pose.pose.pose.orientation.y;
    q.z() = LM.IMU_pose.pose.pose.orientation.z;
    q.w() = LM.IMU_pose.pose.pose.orientation.w;
    t = Eigen::Vector3d( LM.IMU_pose.pose.pose.position.x,LM.IMU_pose.pose.pose.position.y,LM.IMU_pose.pose.pose.position.z);
    T.setIdentity();
    T.rotate(q);
    T.translation() = t;
    LM.T_IMU_preintergration = T;
    LM.PoseCnt++;
    if(LM.PoseCnt>100){
        LM.newPose = true;
    }
}
void gtCallback(const nav_msgs::Odometry::ConstPtr& msg){
    LM.gt_path.header = msg->header;
    geometry_msgs::PoseStamped tmp;
    tmp.pose = msg->pose.pose;
    LM.gt_path.poses.push_back(tmp);

}
int main(int argc, char** argv){
    bool debug = true;
    ros::init(argc, argv, "LiDAR_matching");
    std::cout<<argv[1]<<std::endl;
    LM.time_offset = std::atof(argv[1]);
    ros::NodeHandle nh("~");
    LM.LoadNormalMap("/home/echo/bag/city/ds.pcd");
    ROS_WARN("Map Finish");
    ros::Subscriber sub1 = nh.subscribe("/imu_data", 500, ImuCallback);
    ros::Subscriber sub2 = nh.subscribe("/velodyne_points", 5, PointCallback);
    ros::Subscriber sub3 = nh.subscribe("/initialpose", 1, InitPoseCallback);
    ros::Subscriber sub5 = nh.subscribe("/IMU_Preintegration/IMU_intergration", 1, ImuPoseCallback);
    ros::Subscriber sub4 = nh.subscribe("/gt", 1, gtCallback);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
    ros::Publisher LiDAR_pub = nh.advertise<sensor_msgs::PointCloud2>("/LiDAR_map_Distortion", 1);
    ros::Publisher Local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/LiDAR_map_position", 1);
    ros::Publisher Imu_LiDAR_pub = nh.advertise<nav_msgs::Odometry>("/LiDAR_at_IMU_time", 1);
    ros::Publisher all_path = nh.advertise<nav_msgs::Path>("/all_path", 1);
    ros::Publisher imu_predict_path = nh.advertise<nav_msgs::Path>("/imu_predict_path", 1);
    ros::Publisher Time_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>    ("/Time_cost", 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>    ("/imu_distortion_path", 1);
    ros::Publisher gt_pub = nh.advertise<nav_msgs::Path>    ("/gt_path", 1);
    if(debug){
        LM.T_map.setIdentity();
        LM.T_map_last.setIdentity();
        LM.extrinsicRot.setZero();//(1, 0, 0, 0, -1, 0, 0, 0, -1);
        LM.extrinsicRot(0,0) = 1;
        LM.extrinsicRot(1,1) = 1;
        LM.extrinsicRot(2,2) = 1;
        Eigen::Quaterniond q;
        //start
        q.x() = 3.622718853875994682e-03; q.y() = -6.446913629770278931e-02; q.z() = 9.416124820709228516e-01; q.w() = 3.304492235183715820e-01;
        //q.setIdentity();
        LM.T_map.rotate(q.matrix()) ;
        LM.T_map.translation()<<0,0,0;

//        q.x() = 0.0108444616199; q.y() = -0.12032815069; q.z() = -0.151517733932; q.w() =0.981043279171;
//        LM.T_map.rotate(q.matrix()) ;
//        LM.T_map.translation()<<271.907041259,427.126210513,0;
//        q.x() = 0.0512057840824; q.y() = -0.0337182208896; q.z() =-0.701510906219; q.w() = 0.710016489029;
//        q.normalize();
//        LM.T_map.rotate(q.matrix()) ;
//        LM.T_map.translation()<<513.2,-30.4,0;

        LM.IMU_q.push_back( q );//todo clear
        LM.IMU_p.push_back(LM.T_map.translation());
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
            path_pub.publish(LM.IMU_predict_path);
            all_path.publish(LM.map_path);
            imu_predict_path.publish(LM.imu_constraint_path);
            gt_pub.publish( LM.gt_path);
            LM.newLiDAR = false;
        }
        ros::spinOnce();
        r.sleep();
    }
}
