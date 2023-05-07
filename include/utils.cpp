//
// Created by echo on 23-5-6.
//
#include "Eigen/Eigen"
#include "nav_msgs/Odometry.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"
#include "pcl/common/eigen.h"
Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
{
    double x, y, z, roll, pitch, yaw;
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    return pcl::getTransformation(x, y, z, roll, pitch, yaw);
}