//
// Created by echo on 23-5-7.
//

#include "IMU_preintergration_lib.h"

void IMU_preintergration_lib::process() {
    if(Odom_buffer.size()>1 && new_LiDAR){
        //calc inc
        odom_inc = odom2affine(Odom_buffer.back());

        auto odom_last = Odom_buffer.back();
        Odom_buffer.clear();
        Odom_buffer.push_back(odom_last);

        Eigen::Quaternionf  rotation(odom_inc.rotation());
        lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(rotation.w(),rotation.x(),rotation.y(),rotation.z()),
                                              gtsam::Point3(odom_inc.translation().x(), odom_inc.translation().y(), odom_inc.translation().z()));
        if (systemInitialized == false){
            initlizeGraph();//1.
            return;
        }
        // reset graph for speed
        if (key == 100){
            restGraph();//2.
        }
        if(IMU_buffer.size() !=0 && (IMU_buffer.front().header.stamp.toSec() < (Odom_buffer.back().header.stamp.toSec() - delay))){
            integrateIMU();//3/
            repropagateIMU();//4.
            generatePublishmsg();//5.
        }
    }
}

Eigen::Affine3f IMU_preintergration_lib::odom2affine(nav_msgs::Odometry odom) {
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
}

void IMU_preintergration_lib::restGraph() {
    // get updated noise before reset
    gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
    // reset graph
    resetOptimization();
    // add pose
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
    graphFactors.add(priorPose);
    // add velocity
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
    graphFactors.add(priorVel);
    // add bias
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
    graphFactors.add(priorBias);
    // add values
    graphValues.insert(X(0), prevPose_);
    graphValues.insert(V(0), prevVel_);
    graphValues.insert(B(0), prevBias_);
    // optimize once
    optimizer.update(graphFactors, graphValues);
    graphFactors.resize(0);
    graphValues.clear();

    key = 1;
}

void IMU_preintergration_lib::resetOptimization()
{
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newGraphFactors;
    graphFactors = newGraphFactors;

    gtsam::Values NewGraphValues;
    graphValues = NewGraphValues;
}
//ok
void IMU_preintergration_lib::initlizeGraph() {
    resetOptimization();

    // pop old IMU message
    for (int j = 0; j < IMU_buffer.size(); ++j) {
        if ( IMU_buffer[j].header.stamp.toSec() < Odom_buffer.back().header.stamp.toSec() - delay){
            lastImuT_opt = IMU_buffer[j].header.stamp.toSec();
        } else{
            std::vector<sensor_msgs::Imu>  IMU_q_tmp(IMU_buffer.begin() + j,IMU_buffer.end());
            IMU_buffer = IMU_q_tmp;
            break;
        }
    }

    // initial pose
    prevPose_ = lidarPose.compose(lidar2Imu);
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
    graphFactors.add(priorPose);
    // initial velocity
    prevVel_ = gtsam::Vector3(0, 0, 0);
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
    graphFactors.add(priorVel);
    // initial bias
    prevBias_ = gtsam::imuBias::ConstantBias();
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
    graphFactors.add(priorBias);
    // add values
    graphValues.insert(X(0), prevPose_);
    graphValues.insert(V(0), prevVel_);
    graphValues.insert(B(0), prevBias_);
    // optimize once
    optimizer.update(graphFactors, graphValues);
    graphFactors.resize(0);
    graphValues.clear();

    imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

    key = 1;
    systemInitialized = true;
}

void IMU_preintergration_lib::integrateIMU() {
    bool degenerate =  false;
    bool has_imu=  IMU_buffer[0].header.stamp.toSec() < (Odom_buffer.back().header.stamp.toSec() - delay);;
    for (int j = 0; j < IMU_buffer.size(); ++j) {
        if (IMU_buffer[j].header.stamp.toSec() < (Odom_buffer.back().header.stamp.toSec() - delay)){
//            double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (IMU_buffer[j].header.stamp.toSec() - lastImuT_opt);
            imuIntegratorOpt_->integrateMeasurement(
                    gtsam::Vector3(IMU_buffer[j].linear_acceleration.x, IMU_buffer[j].linear_acceleration.y, IMU_buffer[j].linear_acceleration.z),
                    gtsam::Vector3(IMU_buffer[j].angular_velocity.x,    IMU_buffer[j].angular_velocity.y,    IMU_buffer[j].angular_velocity.z), 0.005);
            lastImuT_opt = IMU_buffer[j].header.stamp.toSec();
        }else{
            std::vector<sensor_msgs::Imu>  IMU_q_tmp(IMU_buffer.begin() + j,IMU_buffer.end());
            IMU_buffer = IMU_q_tmp;
            break;
        }
    }
    // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    graphFactors.add(imu_factor);
    // add imu bias between factor
    graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                                        gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
    // add pose factor
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);//ok
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose,  correctionNoise);
    graphFactors.add(pose_factor);
    // insert predicted values
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(V(key), propState_.v());
    graphValues.insert(B(key), prevBias_);
    // optimize
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step.
    gtsam::Values result = optimizer.calculateEstimate();
    prevPose_  = result.at<gtsam::Pose3>(X(key));
    prevVel_   = result.at<gtsam::Vector3>(V(key));
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));


    std::cerr<<"Bias : "<<prevBias_<<" vel: "<<prevVel_.transpose()<<std::endl;
    // Reset the optimization preintegration object.
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    // check optimization
    if (failureDetection(prevVel_, prevBias_)){
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
        return;
    }
}
bool IMU_preintergration_lib::failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
{
    Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 30)
    {
        ROS_WARN("Large velocity, reset IMU-preintegration!");
        return true;
    }

    Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
    Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
    if (ba.norm() > 1.0 || bg.norm() > 1.0)
    {
        ROS_WARN("Large bias, reset IMU-preintegration!");
        return true;
    }

    return false;
}

void IMU_preintergration_lib::repropagateIMU() {
    prevStateOdom = prevState_;
    prevBiasOdom  = prevBias_;
    // first pop imu message older than current correction data
    double lastImuQT = -1;
    for (int j = 0; j < repropagateIMU_buffer.size(); ++j) {
        if (repropagateIMU_buffer[j].header.stamp.toSec() <  Odom_buffer.back().header.stamp.toSec() - delay){
            lastImuQT = repropagateIMU_buffer[j].header.stamp.toSec();
        } else{
            std::vector<sensor_msgs::Imu>  IMU_q_tmp(repropagateIMU_buffer.begin() + j, repropagateIMU_buffer.end());
            repropagateIMU_buffer = IMU_q_tmp;
        }
    }

    // repropogate
    if (!repropagateIMU_buffer.empty())
    {
        // reset bias use the newly optimized bias
        imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
        // integrate imu message from the beginning of this optimization
        for (int i = 0; i < (int)repropagateIMU_buffer.size(); ++i)
        {
            sensor_msgs::Imu *thisImu = &repropagateIMU_buffer[i];
            double imuTime =repropagateIMU_buffer[i].header.stamp.toSec();
            imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), 0.005);
            lastImuQT = imuTime;
        }
    }

    ++key;
    doneFirstOpt = true;
}

void IMU_preintergration_lib::generatePublishmsg() {
    // integrate this single imu message
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(IMU_buffer.back().linear_acceleration.x,
                                                           IMU_buffer.back().linear_acceleration.y, IMU_buffer.back().linear_acceleration.z),
                                            gtsam::Vector3(IMU_buffer.back().angular_velocity.x,
                                                           IMU_buffer.back().angular_velocity.y,    IMU_buffer.back().angular_velocity.z), 0.005);
    // predict odometry
    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());

    odometry.header.stamp = Odom_buffer.back().header.stamp;
    odometry.header.frame_id ="map";
    odometry.pose.pose.position.x = imuPose.translation().x();
    odometry.pose.pose.position.y = imuPose.translation().y();
    odometry.pose.pose.position.z = imuPose.translation().z();
    odometry.pose.pose.orientation.x = imuPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = imuPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = imuPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = imuPose.rotation().toQuaternion().w();
    Eigen::Vector3d  velocity(currentState.velocity().x(),currentState.velocity().y(),currentState.velocity().z());
    velocity = imuPose.rotation().inverse() * velocity;
    odometry.twist.twist.linear.x = velocity.x();
    odometry.twist.twist.linear.y = velocity.y();
    odometry.twist.twist.linear.z = velocity.z();
    odometry.twist.twist.angular.x = IMU_buffer.back().angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = IMU_buffer.back().angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = IMU_buffer.back().angular_velocity.z + prevBiasOdom.gyroscope().z();
    new_LiDAR = false;
}

void IMU_preintergration_lib::IMUintergation(sensor_msgs::Imu thisImu) {
    if(systemInitialized){
        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), 0.005);
        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
        // publish odometry
        IMU_odometry.header.stamp = thisImu.header.stamp;
        IMU_odometry.header.frame_id = "map";
//    IMU_odometry.child_frame_id = "imu";
        coorected_IMU = thisImu;
        gtsam::Vector3 accel(thisImu.linear_acceleration.x,
                             thisImu.linear_acceleration.y,
                             thisImu.linear_acceleration.z);
        gtsam:: Vector3 omega(thisImu.angular_velocity.x,
                              thisImu.angular_velocity.y,
                              thisImu.angular_velocity.z);
        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);
        auto g = lidarPose.rotation().matrix().inverse() * imuIntegratorImu_->p().getGravity().matrix() ;
        accel = accel + g + prevBiasOdom.accelerometer();
        omega = omega + prevBiasOdom.gyroscope() ;
        coorected_IMU.header.frame_id = "velodyne";
        coorected_IMU.angular_velocity.x = omega.x();
        coorected_IMU.angular_velocity.y = omega.y();
        coorected_IMU.angular_velocity.z = omega.z();
        coorected_IMU.linear_acceleration.x = accel.x();
        coorected_IMU.linear_acceleration.y = accel.y();
        coorected_IMU.linear_acceleration.z = accel.z();

        coorected_IMU.linear_acceleration_covariance[0]= currentState.velocity().x();
        coorected_IMU.linear_acceleration_covariance[1]= currentState.velocity().y();
        coorected_IMU.linear_acceleration_covariance[2]= currentState.velocity().z();

        IMU_odometry.pose.pose.position.x = lidarPose.translation().x();
        IMU_odometry.pose.pose.position.y = lidarPose.translation().y();
        IMU_odometry.pose.pose.position.z = lidarPose.translation().z();
        IMU_odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        IMU_odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        IMU_odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        IMU_odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        coorected_IMU.orientation = IMU_odometry.pose.pose.orientation;

        IMU_odometry.twist.twist.linear.x = currentState.velocity().x();
        IMU_odometry.twist.twist.linear.y = currentState.velocity().y();
        IMU_odometry.twist.twist.linear.z = currentState.velocity().z();
        IMU_odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        IMU_odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        IMU_odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    }
}
