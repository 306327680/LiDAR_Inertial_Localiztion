//
// Created by echo on 23-5-7.
//
#include "IMU_preintergration_lib/IMU_preintergration_lib.h"


ros::Subscriber subImuOdometry;
ros::Subscriber subLaserOdometry;
ros::Publisher pubImuOdometry;
ros::Publisher pubImuPath;
IMU_preintergration_lib IP;
void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& msg){
     IP.lidarOdomAffine = IP.odom2affine(*msg);
     IP.lidarOdomTime = msg->header.stamp.toSec();
     IP.Odom_buffer.push_back(*msg);
     IP.new_LiDAR = true;

}
void imuOdometryHandler(const sensor_msgs::Imu::ConstPtr& msg){
    sensor_msgs::Imu msg_t = *msg;
    msg_t.angular_velocity.x = -msg_t.angular_velocity.x;
    msg_t.angular_velocity.z = -msg_t.angular_velocity.z;
    msg_t.linear_acceleration.x = -msg_t.linear_acceleration.x ;
    msg_t.linear_acceleration.z = -msg_t.linear_acceleration.z ;
    IP.IMU_buffer.push_back(msg_t);
    IP.repropagateIMU_buffer.push_back(msg_t);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "IMU_preintergration");
    ros::NodeHandle nh("~");
    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/LiDAR_map_position", 5,  lidarOdometryHandler,ros::TransportHints().tcpNoDelay());
    subImuOdometry   = nh.subscribe<sensor_msgs::Imu>("/imu/data_raw",   2000,  imuOdometryHandler ,ros::TransportHints().tcpNoDelay());

    pubImuOdometry   = nh.advertise<nav_msgs::Odometry>("IMU_odometry", 2000);
    pubImuPath       = nh.advertise<nav_msgs::Path>    ("IMU_odometry_path", 1);
    ros::Rate r(200);
    while(ros::ok()){
        IP.process();
        pubImuOdometry.publish(IP.odometry);
        ros::spinOnce();
        r.sleep();
    }
}
