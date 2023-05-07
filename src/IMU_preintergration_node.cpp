//
// Created by echo on 23-5-7.
//
#include "IMU_preintergration_lib/IMU_preintergration_lib.h"


ros::Subscriber subImuOdometry;
ros::Subscriber subLaserOdometry;
ros::Publisher pubImuOdometry;
ros::Publisher pubImuPath;

void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& msg){

}
void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& msg){

}

int main(int argc, char** argv){
    ros::init(argc, argv, "IMU_preintergration");
    ros::NodeHandle nh("~");
    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("LiDAR_map_position", 5,  lidarOdometryHandler,ros::TransportHints().tcpNoDelay());
    subImuOdometry   = nh.subscribe<nav_msgs::Odometry>("IMU_map_position",   2000,  imuOdometryHandler ,ros::TransportHints().tcpNoDelay());

    pubImuOdometry   = nh.advertise<nav_msgs::Odometry>("IMU_odometry", 2000);
    pubImuPath       = nh.advertise<nav_msgs::Path>    ("IMU_odometry_path", 1);
    ros::Rate r(200);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}
