//
// Created by echo on 23-5-7.
//
#include "IMU_preintergration_lib/IMU_preintergration_lib.h"


ros::Subscriber subImuOdometry;
ros::Subscriber subLaserOdometry;
ros::Publisher pubImuOdometry;
ros::Publisher pubImuIntergration;
ros::Publisher pubImuCorr;
ros::Publisher pubFusionPath;
ros::Publisher speed_pub;
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
    IP.IMUintergation(msg_t);
    //publish TF
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    pubImuIntergration.publish(IP.IMU_odometry);
    pubImuCorr.publish(IP.coorected_IMU);
    transform.setOrigin(tf::Vector3(IP.IMU_odometry.pose.pose.position.x,IP.IMU_odometry.pose.pose.position.y,IP.IMU_odometry.pose.pose.position.z));
    tf::Quaternion q(IP.IMU_odometry.pose.pose.orientation.x,IP.IMU_odometry.pose.pose.orientation.y,
                     IP.IMU_odometry.pose.pose.orientation.z,IP.IMU_odometry.pose.pose.orientation.w);
    transform.setRotation(q);
    Eigen::Vector3d speed(IP.coorected_IMU.linear_acceleration_covariance[0],IP.coorected_IMU.linear_acceleration_covariance[1],IP.coorected_IMU.linear_acceleration_covariance[2]);
    speed =  Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z()).inverse().matrix() * speed ;
    IP.speed_from_imu = IP.coorected_IMU;
    IP.speed_from_imu.linear_acceleration.x =  speed.x();
    IP.speed_from_imu.linear_acceleration.y =  speed.y();
    IP.speed_from_imu.linear_acceleration.z =  speed.z();
    speed_pub.publish(IP.speed_from_imu);
    br.sendTransform(tf::StampedTransform(transform, IP.IMU_odometry.header.stamp, "map", "velodyne"));
    //TF ok
    IP.new_IMU = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh("~");
    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/LiDAR_map_position", 5,  lidarOdometryHandler,ros::TransportHints().tcpNoDelay());
    subImuOdometry   = nh.subscribe<sensor_msgs::Imu>("/imu/data_raw",   2000,  imuOdometryHandler ,ros::TransportHints().tcpNoDelay());

    pubImuOdometry   = nh.advertise<nav_msgs::Odometry>("IMU_odometry", 2000);
    pubImuIntergration = nh.advertise<nav_msgs::Odometry>    ("IMU_intergration", 1);
    pubImuCorr = nh.advertise<sensor_msgs::Imu>    ("Imu_corrected", 1);
    pubFusionPath = nh.advertise<nav_msgs::Path>    ("Fusion_path", 1);
    speed_pub = nh.advertise<sensor_msgs::Imu>("/current_speed",1);
    ros::Rate r(500);
    while(ros::ok()){
        if( IP.new_LiDAR ){
            IP.process();
            pubImuOdometry.publish(IP.odometry);
            pubFusionPath.publish(IP.Fusion_path);
        }
        ros::spinOnce();
        r.sleep();
    }
}
