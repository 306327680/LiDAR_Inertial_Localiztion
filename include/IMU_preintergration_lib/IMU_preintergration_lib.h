//
// Created by echo on 23-5-7.
//

#ifndef LIDAR_INERTIA_LOCALIZTION_IMU_PREINTERGRATION_LIB_H
#define LIDAR_INERTIA_LOCALIZTION_IMU_PREINTERGRATION_LIB_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "Eigen/Eigen"
#include "pcl/common/eigen.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class IMU_preintergration_lib {
public:
    IMU_preintergration_lib(){
        extTrans.setZero();
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(9.81);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(1e-04, 2);       // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(1e-05  , 2);     // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(5e-5, 2);        // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << -0.0172629, 0.00221303, -0.00185318, 0.000478473, 0.000513444, 0.000380275).finished()); // assume zero initial bias
        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 5e-1, 5e-1, 5e-1, 5e-1, 5e-1, 5e-1).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e3); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.02, 0.02, 0.02, 0.05, 0.05, 0.05).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << 6e-06, 6e-06, 6e-06, 3e-06, 3e-06, 3e-06).finished();
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization
    }
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom);
    void IMUintergation(sensor_msgs::Imu thisImu);
    void process();
    nav_msgs::Odometry odometry;
    nav_msgs::Odometry IMU_odometry;
    nav_msgs::Path  Fusion_path;
    sensor_msgs::Imu  coorected_IMU;
    sensor_msgs::Imu  speed_from_imu;
    std::vector<sensor_msgs::Imu> IMU_buffer;
    std::vector<sensor_msgs::Imu> repropagateIMU_buffer;
    std::vector<nav_msgs::Odometry> Odom_buffer;

    Eigen::Affine3f lidarOdomAffine;
    double lidarOdomTime = -1;
    double delay = 0;
    Eigen::Vector3d extTrans = Eigen::Vector3d(0.05,0,-0.0);
    bool new_LiDAR = false;
    bool new_IMU = false;
private:
    void resetOptimization();
    void initlizeGraph();
    void restGraph();
    void integrateIMU();
    void repropagateIMU();
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);
    void generatePublishmsg();

    bool systemInitialized = false;
    Eigen::Affine3f odom_inc;
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    // T_bl: tramsform points from lidar frame to imu frame
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    // T_lb: tramsform points from imu frame to lidar frame
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));


    int key = 1;
    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    gtsam::Pose3 lidarPose;
};


#endif //LIDAR_INERTIA_LOCALIZTION_IMU_PREINTERGRATION_LIB_H
