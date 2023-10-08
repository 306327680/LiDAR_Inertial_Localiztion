//
// Created by echo on 23-5-2.
//

#include "LiDAR_matching_lib.h"

void LiDAR_matching_lib::process() {
    //debug msg
    Time_used.values.clear();
    LocalMap.clear();
    auto _now_ms_total = std::chrono::high_resolution_clock ::now();
    if (InitPoseBool) {
        icp.transformation = T_map.matrix().cast<float>();
        handleMessage();
        InitPoseBool = false;
        InitPoseCome = true;
    }
    if (InitPoseCome && newIMU && newLiDAR) {
        if (vlp_pcd.size() == 0) {//first cloud
            FrameTime = Point_raw.header.stamp.toSec();
            pcl::PCLPointCloud2 pcl_pc2;
            if (Usemid360) {
                VLPPoint temp;
                pcl::PointCloud<mid360> midconvert;
                pcl::fromROSMsg(Point_raw, midconvert);
                vlp_pcd.clear();
                for (int i = 0; i < midconvert.size(); ++i) {
                    temp.x = midconvert[i].x;
                    temp.y = midconvert[i].y;
                    temp.z = midconvert[i].z;
                    temp.intensity = midconvert[i].intensity;
                    temp.ring = midconvert[i].ring;
                    temp.time = -0.1 + 0.1 * i / midconvert.size();
                    vlp_pcd.push_back(temp);
                }
            } else {
                pcl::fromROSMsg(Point_raw, vlp_pcd);
            }
        }
        AccumulateImu();
        if (imuReady) {
            vlp_ds_pcd.clear();
            auto _now_ms = std::chrono::high_resolution_clock ::now();
            InputDownSample();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _now_ms);
            saveProcessTime("0. InputDownSample Time: ", duration.count()/1e9);

            _now_ms = std::chrono::high_resolution_clock ::now();
            ImuDistortion(vlp_pcd.front().time, vlp_pcd.back().time); //2. Input distortion
            duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _now_ms);
            saveProcessTime("1. Distortion Time: ", duration.count()/1e9);

            _now_ms = std::chrono::high_resolution_clock ::now();
            genLocalMap();
            duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _now_ms);
            saveProcessTime("2. map Time: ", duration.count()/1e9);
            //3. ICP


            _now_ms = std::chrono::high_resolution_clock ::now();
            registrion(vlp_ds_pcd, LocalMap);
            handleMessage();
            duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _now_ms);
            saveProcessTime("3. icp Time: ", duration.count()/1e9);

            pcl::fromROSMsg(Point_raw, vlp_pcd); //untill imu queue is ok
            FrameTime = Point_raw.header.stamp.toSec();
            T_map_last = T_map;
            imuReady = false;
        } else{
            ROS_WARN("No matching IMU");
        }
    }

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _now_ms_total);
    saveProcessTime("4. Total Time: ", duration.count()/1e9);
    if((float)(duration.count()/1e9)>0.1){
        ROS_WARN("Time cost too high! ");
    }
    std::cout << "Process Time: " << duration.count()/1e9<< std::endl;
}

void LiDAR_matching_lib::LoadNormalMap(std::string map_path) {

    pcl::PCLPointCloud2 pcl_frame;
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile(map_path, mls_points);
    pcl::toPCLPointCloud2(mls_points, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, mls_map);
    mls_map.header.stamp = ros::Time::now();
    mls_map.header.frame_id = "map";
    *mls_ptr = mls_points;
    kdtree.setInputCloud(mls_ptr);
    T_map.setIdentity();
    icp.SetNormalICP();
    gyro_last.setZero();
//    LiDAR_map_last.header.stamp = ros::Time::now();
}

void LiDAR_matching_lib::LoadMap(std::string map_path) {
//    kdtree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>());

    pcl::PCLPointCloud2 pcl_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile(map_path, *cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(1);
    mls.process(mls_points);
    pcl::toPCLPointCloud2(mls_points, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, mls_map);
    mls_map.header.stamp = ros::Time::now();
    mls_map.header.frame_id = "map";
    *mls_ptr = mls_points;
    kdtree.setInputCloud(mls_ptr);
    T_map.setIdentity();
    T_IMU_predict.setIdentity();
    icp.SetNormalICP();
}

//todo 1. accumulate rotate from last pose 2.pop IMU until >=FrameTime+first_point_time 3. rotate point cloud
void LiDAR_matching_lib::ImuDistortion(double first_point_time, double last_point_time) {

    std::vector<sensor_msgs::Imu> sync_imu;
    //XXX IMU_q starts from q[last.back()]<<T[last_scan first cloud time]<<q[0]<<q[1]<<...<<q[back]<<T[last_scan last cloud time] time

    if (imuReady) {
        Eigen::Quaterniond q_init;
        Eigen::Quaterniond q_last;
        Eigen::Vector3d  p_init;
        Eigen::Vector3d  p_last;
        findRotation(FrameTime + time_offset, q_init);//all transform to Frame time
        findTrans(FrameTime + time_offset , p_init);
        IMU_p_latest = p_init;
        IMU_q_latest = q_init;
        Eigen::Quaterniond q;
        Eigen::Vector3d  p;
        for (int i = 0; i < vlp_ds_pcd.size(); ++i) {
            findRotation(FrameTime + vlp_ds_pcd[i].intensity + time_offset, q);
            findTrans(FrameTime + vlp_ds_pcd[i].intensity + time_offset , p);
            Eigen::Vector3d point(vlp_ds_pcd[i].x, vlp_ds_pcd[i].y, vlp_ds_pcd[i].z);
//            point = q_init.matrix().inverse() * q.matrix() * point + Eigen::Vector3d(icp.increase(0, 3),
//                                                                                     icp.increase(1, 3),
//                                                                                     icp.increase(2, 3)) *
//                                                                     (vlp_ds_pcd[i].intensity) * 10;//10hz
            point = q_init.matrix().inverse() * q.matrix() *  point + q_init.matrix().inverse() * (p - p_init);
            vlp_ds_pcd[i].x = point.x();
            vlp_ds_pcd[i].y = point.y();
            vlp_ds_pcd[i].z = point.z();
        }
        if(LastFrameTime != 0){
            findRotation(LastFrameTime + time_offset,q_last);
            findTrans(LastFrameTime + time_offset , p_last);
            T_IMU_predict.setIdentity();
            T_IMU_predict.rotate( q_init.matrix());
           // T_IMU_predict.rotate( q_init.matrix());
            T_IMU_predict.translate(q_last.matrix().inverse()*(p_init-p_last));
            T_map = T_map.rotate( q_init.matrix());
            T_map = T_map.rotate( q_last.matrix().inverse());
//            T_map = T_map.translate(Eigen::Vector3d (icp.increase(0,3),icp.increase(1,3),icp.increase(2,3)));
            T_map = T_map.translate(q_last.matrix().inverse() * (p_init-p_last));
            //T_map.translation() = p_init;//use imu positiom not work;
        }
        if(newPose){
            T_IMU_predict=T_IMU_preintergration;
            T_map = T_IMU_preintergration;
        }
        IMU_pose_latest.setIdentity();
        IMU_pose_latest.rotate(T_map.rotation());
        IMU_pose_latest.translate(p_init);
//        std::cout<<"IMU predict trans: : " << (q_last.matrix().inverse()*(p_init-p_last)).transpose();
        LiDAR_at_IMU_Time.header.stamp = ros::Time(FrameTime);
        LiDAR_at_IMU_Time.header.frame_id = "map";
        LiDAR_at_IMU_Time.pose.pose.position.x = p_init.x();
        LiDAR_at_IMU_Time.pose.pose.position.y = p_init.y();
        LiDAR_at_IMU_Time.pose.pose.position.z = p_init.z();
        LiDAR_at_IMU_Time.pose.pose.orientation.x = q_init.x();
        LiDAR_at_IMU_Time.pose.pose.orientation.y = q_init.y();
        LiDAR_at_IMU_Time.pose.pose.orientation.z = q_init.z();
        LiDAR_at_IMU_Time.pose.pose.orientation.w = q_init.w();
        geometry_msgs::PoseStamped tmp;
        Eigen::Quaterniond qq ;
        qq = T_map.rotation();
        tmp.pose.position.x = T_map.translation().x();
        tmp.pose.position.y = T_map.translation().y();
        tmp.pose.position.z = T_map.translation().z();
        tmp.pose.orientation.x = qq.x();
        tmp.pose.orientation.y = qq.y();
        tmp.pose.orientation.z = qq.z();
        tmp.pose.orientation.w = qq.w();
        imu_constraint_path.poses.push_back(tmp);
    }
}

void LiDAR_matching_lib::registrion(pcl::PointCloud<pcl::PointXYZI> source,
                                    pcl::PointCloud<pcl::PointNormal> target) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointNormal> map_T_last;
    pcl::PointCloud<pcl::PointXYZI> local_map;
    *temp = source;
    pcl::transformPointCloudWithNormals(target, map_T_last, T_map.matrix().inverse().cast<float>()); //fix bug?
    icp.transformation = Eigen::Matrix4f::Identity();
    icp.initialPose =T_IMU_predict.matrix().cast<float>();
//    icp.IMU_p_latest =   (IMU_pose_latest * T_map.inverse()).translation() ;
//    std::cerr<<"icp.IMU_p_latest: "<<IMU_p_latest<<T_map.translation() <<std::endl;
//    icp.IMU_p_latest = Eigen::Vector3d::Zero();
    icp.icp(temp, map_T_last);

    T_map = T_map.matrix() * icp.transformation.cast<double>();//reduce the prediction error
    pcl::transformPointCloud(source, local_map, T_map.matrix().cast<float>());
    Transfer_local_point = local_map;
}

void LiDAR_matching_lib::handleMessage() {
    //4.pub
    //1.cloud under map coord
    pcl::PCLPointCloud2 pcl_frame;
    pcl::toPCLPointCloud2(Transfer_local_point, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, LiDAR_Map);
    LiDAR_Map.header = Point_raw.header;
    LiDAR_Map.header.frame_id = "map";


    pcl::toPCLPointCloud2(LocalMap, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, LocalMapPC2);
    LocalMapPC2.header.stamp = Point_raw.header.stamp;
    LocalMapPC2.header.frame_id = "map";


    LiDAR_map.header.stamp = Point_raw.header.stamp;
    LiDAR_map.header.frame_id = "map";
    LiDAR_map.pose.pose.position.x = T_map.translation().x();
    LiDAR_map.pose.pose.position.y = T_map.translation().y();
    LiDAR_map.pose.pose.position.z = T_map.translation().z();
    Eigen::Quaterniond q;
    q = T_map.rotation();
    auto increase = (T_map_last.matrix().inverse()) * T_map.matrix();

    LiDAR_map.pose.pose.orientation.x = q.x();
    LiDAR_map.pose.pose.orientation.y = q.y();
    LiDAR_map.pose.pose.orientation.z = q.z();
    LiDAR_map.pose.pose.orientation.w = q.w();
    LiDAR_map.twist.twist.linear.x = increase(0, 3) / (FrameTime - LastFrameTime);
    LiDAR_map.twist.twist.linear.y = increase(1, 3) / (FrameTime - LastFrameTime);
    LiDAR_map.twist.twist.linear.z = increase(2, 3) / (FrameTime - LastFrameTime);
    LiDAR_map.twist.twist.angular.x = icp.increase(0,3);//debug for icp result
    LiDAR_map.twist.twist.angular.y = icp.increase(1,3);
    LiDAR_map.twist.twist.angular.z = icp.increase(2,3);

    for (int i = 0; i < icp.covariance_matrix.size(); ++i) {
        LiDAR_map.pose.covariance[i] = icp.covariance_matrix(i);
    }
    map_path.header.stamp = Point_raw.header.stamp;
    map_path.header.frame_id = "map";
    geometry_msgs::PoseStamped tmp;
    tmp.pose = LiDAR_map.pose.pose;
    map_path.poses.push_back(tmp);

    imu_constraint_path.header = map_path.header;
    //from IMU velocity predicted lidar pose
    geometry_msgs::PoseStamped odom;
    IMU_predict_path.poses.clear();
    IMU_predict_path.header.stamp = Point_raw.header.stamp;
    IMU_predict_path.header.frame_id = "map";
    odom.pose =LiDAR_at_IMU_Time.pose.pose;
    IMU_predict_path.poses.push_back(odom);
    q.setIdentity();
    if (IMU_q.size() > 800) {
        std::vector<double> IMU_Time_tmp(IMU_Time.end() - 800, IMU_Time.end());
        std::vector<Eigen::Quaterniond> IMU_q_tmp(IMU_q.end() - 800, IMU_q.end());
        std::vector<Eigen::Vector3d> IMU_p_tmp(IMU_p.end() - 800, IMU_p.end());
        std::vector<Eigen::Vector3d> IMU_v_tmp(IMU_v.end() - 800, IMU_v.end());
        IMU_p = IMU_p_tmp;
        IMU_q = IMU_q_tmp;
        IMU_v = IMU_v_tmp;
        IMU_Time = IMU_Time_tmp;
    }
    T_map_last = T_map;
    LastFrameTime = FrameTime;
}

void LiDAR_matching_lib::InputDownSample() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr vlp_pcd_ds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    vlp_ds_pcd.clear();
    for (int i = 0; i < vlp_pcd.size(); i = i + 3) {
        pcl::PointXYZI point_xyzi;
        point_xyzi.x = vlp_pcd[i].x;
        point_xyzi.y = vlp_pcd[i].y;
        point_xyzi.z = vlp_pcd[i].z;
        point_xyzi.intensity = vlp_pcd[i].time;
        if (sqrt(vlp_pcd[i].x * vlp_pcd[i].x + vlp_pcd[i].y * vlp_pcd[i].y +vlp_pcd[i].z * vlp_pcd[i].z) < 40) {
            vlp_pcd_ds_ptr->push_back(point_xyzi);
        }
    }
    RS.setInputCloud(vlp_pcd_ds_ptr);
    RS.setSample(1000);
    RS.filter(vlp_ds_pcd);
/*    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor1;
    sor1.setInputCloud(vlp_pcd_ds_ptr);
    sor1.setMeanK(5);
    sor1.setStddevMulThresh(1.0);
    sor1.filter(*vlp_pcd_ds_ptr);*/
/*    sor.setInputCloud(vlp_pcd_ds_ptr);
    sor.setLeafSize(3, 3, 3);
    sor.filter(vlp_ds_pcd);*/
}

void LiDAR_matching_lib::findRotation(double pointTime, Eigen::Quaterniond &Q) {
    int index = 0;
    for (int i = 0; i < IMU_Time.size(); ++i) {
        index = i;
        if (pointTime < IMU_Time[i]) {
            break;
        }
    }
    if (index == IMU_Time.size() - 1) {
        Q = IMU_q.back();
    } else {
        double alpha = (pointTime - IMU_Time[index]) / (IMU_Time[index + 1] - IMU_Time[index]);
        Q = IMU_q[index].slerp(alpha, IMU_q[index + 1]);
    }
}
//init value from IMU


void LiDAR_matching_lib::genLocalMap() {
    auto _now_ms = std::chrono::high_resolution_clock ::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _now_ms);

    Eigen::Affine3d curr_pose;
    curr_pose = T_map;

    pcl::PointCloud<pcl::PointXYZI> vlp_pcd_range;
    std::vector<int> indices_unique;

    for (int i = 0; i < vlp_ds_pcd.size(); i = i + 1) {
        pcl::PointXYZI point_xyzi;
        if (sqrt(vlp_ds_pcd[i].x * vlp_ds_pcd[i].x + vlp_ds_pcd[i].y * vlp_ds_pcd[i].y +
                 vlp_ds_pcd[i].z * vlp_ds_pcd[i].z) < 50) {
            point_xyzi.x = vlp_ds_pcd[i].x;
            point_xyzi.y = vlp_ds_pcd[i].y;
            point_xyzi.z = vlp_ds_pcd[i].z;
            point_xyzi.intensity = sqrt(vlp_ds_pcd[i].x * vlp_ds_pcd[i].x + vlp_ds_pcd[i].y * vlp_ds_pcd[i].y +
                                         vlp_ds_pcd[i].z * vlp_ds_pcd[i].z);
            vlp_pcd_range.push_back(point_xyzi);
        }
    }
    vlp_ds_pcd = vlp_pcd_range;
    std::vector<int> indices; // 存储查询近邻点索引
    Eigen::Vector3d point;
    for (int i = 0; i < vlp_pcd_range.size(); ++i) {
        std::vector<int> indices; // 存储查询近邻点索引
        Eigen::Vector3d point;
        point = Eigen::Vector3d(vlp_pcd_range[i].x, vlp_pcd_range[i].y, vlp_pcd_range[i].z);
        point = curr_pose.rotation() * point + curr_pose.translation();
        pcl::PointNormal temp;
        temp.x = point.x();
        temp.y = point.y();
        temp.z = point.z();
        kdtree.nearestKSearch(temp, 20, indices, distances);
        //kdtree.nearestKSearch(temp, 3.0 * ceil(vlp_pcd_range[i].intensity), indices, distances);
        //kdtree.radiusSearch(temp, 0.75, indices, distances);
        for (int j = 0; j < indices.size(); ++j) {
            if(distances[j]<0.7){
                indices_unique.push_back(indices[j]);
            }
        }
    }
    saveProcessTime("2.1 Kdtree Time: ", duration.count()/1e9);
    _now_ms = std::chrono::high_resolution_clock ::now();
//    for (int i = 0; i <indices_last.size() ; ++i) {
//        indices_unique.push_back(indices_last[i]);
//    }
    std::sort(indices_unique.begin(), indices_unique.end());
    indices_unique.erase(std::unique(indices_unique.begin(), indices_unique.end()), indices_unique.end());
    pcl::PointCloud<pcl::PointNormal>::Ptr local_candidate(new pcl::PointCloud<pcl::PointNormal> );
    for (int j = 0; j < indices_unique.size(); ++j) {
        local_candidate->push_back(mls_points[indices_unique[j]]);
    }
    RS2.setInputCloud(local_candidate);
    RS2.setSample(7200);
    RS2.setSample(7200);
    RS2.filter(LocalMap);
    indices_last = indices_unique;
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _now_ms);
    saveProcessTime("2.2 from indices find map Time: ", duration.count()/1e9);
    _now_ms = std::chrono::high_resolution_clock ::now();
}

void LiDAR_matching_lib::AccumulateImu() {
    for (int i = 0; i < ImuQueue.size(); ++i) {
        Eigen::Vector3d gyro;
        Eigen::Vector3d acc;
        Eigen::Vector3d vel;
        Eigen::Vector3d position;
//            gyro[0] = -ImuQueue[i].angular_velocity.x;
//            gyro[1] =  ImuQueue[i].angular_velocity.y;
//            gyro[2] = -ImuQueue[i].angular_velocity.z;
        gyro[0] = ImuQueue[i].angular_velocity.x;
        gyro[1] = ImuQueue[i].angular_velocity.y;
        gyro[2] = ImuQueue[i].angular_velocity.z;
        acc[0] = ImuQueue[i].linear_acceleration.x;
        acc[1] = ImuQueue[i].linear_acceleration.y;
        acc[2] = ImuQueue[i].linear_acceleration.z;
        vel[0] = ImuQueue[i].linear_acceleration_covariance[0]; //todo velocity under map
        vel[1] = ImuQueue[i].linear_acceleration_covariance[1];
        vel[2] = ImuQueue[i].linear_acceleration_covariance[2];
        position[0] = ImuQueue[i].linear_acceleration_covariance[3]; //position under map
        position[1] = ImuQueue[i].linear_acceleration_covariance[4];
        position[2] = ImuQueue[i].linear_acceleration_covariance[5];
        IMU_v.push_back(vel);
        Eigen::Quaterniond dq;
        Eigen::Vector3d dp;

        Eigen::Vector3d dtheta_half = (gyro_last / 4.0 + gyro / 4.0) * IMU_period_time;
        gyro_last = gyro;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        if (IMU_q.empty()) {
            dq.setIdentity();
            dp.setZero();
            IMU_q.push_back(dq);
        } else {
            dq = IMU_q.back() * dq;
            dq.normalize();
            IMU_q.push_back(dq);
        }
        Eigen::Vector3d acc_w = IMU_q.back() * (acc);
        dp = IMU_p.back() + vel *  IMU_period_time + 0.5 * IMU_period_time * IMU_period_time * acc_w;
        IMU_p.push_back(position);
        IMU_Time.push_back(ImuQueue[i].header.stamp.toSec());
    }
    ImuQueue.clear();
    if (IMU_Time.back() >= FrameTime + vlp_pcd.back().time + time_offset) {
        imuReady = true;
    } else {
        imuReady = false;
    }
}

void LiDAR_matching_lib::saveProcessTime(std::string name, double value) {
    diagnostic_msgs::KeyValue temp_time;
    temp_time.key = name;
    temp_time.value = std::to_string(value);
    Time_used.values.push_back(temp_time);
}

void LiDAR_matching_lib::findTrans(double pointTime, Eigen::Vector3d &T) {
    int index = 0;
    for (int i = 0; i < IMU_Time.size(); ++i) {
        index = i;
        if (pointTime < IMU_Time[i]) {
            break;
        }
    }
    if (index == IMU_Time.size() - 1) {
        T = IMU_p.back();
    } else {
        double alpha = (pointTime - IMU_Time[index]) / (IMU_Time[index + 1] - IMU_Time[index]);
        T = IMU_p[index]+ alpha * (IMU_p[index + 1]-IMU_p[index]);
    }
}


