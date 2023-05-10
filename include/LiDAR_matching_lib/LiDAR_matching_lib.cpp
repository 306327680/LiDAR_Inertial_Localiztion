//
// Created by echo on 23-5-2.
//

#include "LiDAR_matching_lib.h"

void LiDAR_matching_lib::process() {
    //debug msg
    ros::Time processTime;
    ros::Time processTime_total;
    processTime_total = ros::Time::now();
    Time_used.values.clear();
    LocalMap.clear();

    if(InitPoseBool){
        icp.transformation  = T_map.matrix().cast<float>();
        InitPoseBool = false;
        InitPoseCome = true;
    }
    if(InitPoseCome && newIMU && newLiDAR){
        if(vlp_pcd.size() == 0){//first cloud
            FrameTime = Point_raw_queue.front().header.stamp.toSec();
            pcl::PCLPointCloud2 pcl_pc2;
            if (Usemid360){
                VLPPoint temp;
                pcl::PointCloud<mid360> midconvert;
                pcl::fromROSMsg(Point_raw_queue.front(),midconvert);
                vlp_pcd.clear();
                for (int i = 0; i < midconvert.size(); ++i) {
                    temp.x = midconvert[i].x;
                    temp.y = midconvert[i].y;
                    temp.z = midconvert[i].z;
                    temp.intensity = midconvert[i].intensity;
                    temp.ring = midconvert[i].ring;
                    temp.time = -0.1 + 0.1*i/midconvert.size();
                    vlp_pcd.push_back(temp);
                }
            } else{
                pcl::fromROSMsg(Point_raw_queue.front(),vlp_pcd);
            }
        }
        AccumulateImu();
        if(imuReady){
            vlp_ds_pcd.clear();
            processTime = ros::Time::now();
            InputDownSample();
            ImuDistortion(vlp_pcd.front().time,vlp_pcd.back().time); //2. Input distortion
            saveProcessTime("1. Distortion Time: ",processTime.toSec()-ros::Time::now().toSec());
            processTime = ros::Time::now();
            genLocalMap( );
            saveProcessTime("2. map Time: ",processTime.toSec()-ros::Time::now().toSec());
            //3. ICP
            processTime = ros::Time::now();
            registrion(vlp_ds_pcd,LocalMap);
            handleMessage();
            saveProcessTime("3. icp Time: ",processTime.toSec()-ros::Time::now().toSec());
            pcl::fromROSMsg(Point_raw_queue.front(),vlp_pcd); //untill imu queue is ok
            FrameTime = Point_raw_queue.front().header.stamp.toSec();
            Point_raw_queue.pop();
            imuReady = false;
        }
    }
    saveProcessTime("4. Total Time: ",processTime_total.toSec()-ros::Time::now().toSec());
    std::cout<<"Process Time: "<<processTime_total.toSec()-ros::Time::now().toSec()<<std::endl;
}
void LiDAR_matching_lib::LoadNormalMap(std::string map_path) {
    kdtree.reset(new pcl::search::KdTree<pcl::PointNormal>());

    pcl::PCLPointCloud2 pcl_frame;
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile (map_path, mls_points);

    pcl::toPCLPointCloud2(mls_points, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, mls_map);
    mls_map.header.stamp = ros::Time::now();
    mls_map.header.frame_id = "map";
    *mls_ptr = mls_points;
    kdtree->setInputCloud(mls_ptr);
    T_map.setIdentity();
    icp.SetNormalICP();
    gyro_last.setZero();
//    LiDAR_map_last.header.stamp = ros::Time::now();
}
void LiDAR_matching_lib::LoadMap(std::string map_path) {
    kdtree.reset(new pcl::search::KdTree<pcl::PointNormal>());

    pcl::PCLPointCloud2 pcl_frame;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile (map_path, *cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);
    mls.setInputCloud (cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (1);
    mls.process (mls_points);
    pcl::toPCLPointCloud2(mls_points, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, mls_map);
    mls_map.header.stamp = ros::Time::now();
    mls_map.header.frame_id = "map";
    *mls_ptr = mls_points;
    kdtree->setInputCloud(mls_ptr);
    T_map.setIdentity();
    icp.SetNormalICP();
}
//todo 1. accumulate rotate from last pose 2.pop IMU until >=FrameTime+first_point_time 3. rotate point cloud
void LiDAR_matching_lib::ImuDistortion(double first_point_time,double last_point_time) {

    std::vector<sensor_msgs::Imu> sync_imu;
    //XXX IMU_q starts from q[last.back()]<<T[last_scan first cloud time]<<q[0]<<q[1]<<...<<q[back]<<T[last_scan last cloud time] time

    if(imuReady){
        Eigen::Quaterniond q_init;
        Eigen::Quaterniond q_last;

        findRotation(FrameTime + time_offset,q_init);//all transform to Frame time
        Eigen::Quaterniond q;
        for (int i = 0; i < vlp_ds_pcd.size(); ++i) {
            findRotation(FrameTime + vlp_ds_pcd[i].intensity + time_offset,q);
            Eigen::Vector3d point(vlp_ds_pcd[i].x,vlp_ds_pcd[i].y,vlp_ds_pcd[i].z);
            point =   q_init.matrix().inverse() * q.matrix()  * point + Eigen::Vector3d (icp.increase(0,3),
                                                                                         icp.increase(1,3),
                                                                                         icp.increase(2,3)) * (vlp_ds_pcd[i].intensity)*10;//10hz
            vlp_ds_pcd[i].x = point.x();
            vlp_ds_pcd[i].y = point.y();
            vlp_ds_pcd[i].z = point.z();
        }
        if(LastFrameTime != 0){
            findRotation(LastFrameTime + time_offset,q_last);
            T_map = T_map.rotate( q_init.matrix());
            T_map = T_map.rotate( q_last.matrix().inverse());
            T_map = T_map.translate(Eigen::Vector3d (icp.increase(0,3),icp.increase(1,3),icp.increase(2,3)));
        }
        LiDAR_at_IMU_Time.header.stamp = ros::Time(FrameTime);
        LiDAR_at_IMU_Time.header.frame_id = "/map";
        LiDAR_at_IMU_Time.pose.pose.orientation.x = q_init.x();
        LiDAR_at_IMU_Time.pose.pose.orientation.y = q_init.y();
        LiDAR_at_IMU_Time.pose.pose.orientation.z = q_init.z();
        LiDAR_at_IMU_Time.pose.pose.orientation.w = q_init.w();
    }
}

void LiDAR_matching_lib::registrion(pcl::PointCloud<pcl::PointXYZI> source,
                                    pcl::PointCloud<pcl::PointNormal> target) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointNormal> map_T_last;
    pcl::PointCloud<pcl::PointXYZI> local_map;
    *temp = source;
    pcl::transformPointCloud(target, map_T_last, T_map.matrix().inverse().cast<float>());
    icp.transformation = Eigen::Matrix4f::Identity();
    icp.icp(temp,map_T_last);

    T_map = T_map.matrix() * icp.transformation.cast<double>();
    pcl::transformPointCloud(source, local_map, T_map.matrix().cast<float>());
    Transfer_local_point = local_map;
}

void LiDAR_matching_lib::handleMessage() {
    //4.pub
    //1.cloud under map coord
    pcl::PCLPointCloud2 pcl_frame;
    pcl::toPCLPointCloud2(Transfer_local_point, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, LiDAR_Map);
    LiDAR_Map.header = Point_raw_queue.front().header;
    LiDAR_Map.header.frame_id = "/map";


    pcl::toPCLPointCloud2(LocalMap, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, LocalMapPC2);
    LocalMapPC2.header.stamp = Point_raw_queue.front().header.stamp;
    LocalMapPC2.header.frame_id = "/map";


    LiDAR_map.header.stamp = Point_raw_queue.front().header.stamp;
    LiDAR_map.header.frame_id = "/map";
    LiDAR_map.pose.pose.position.x = T_map.translation().x();
    LiDAR_map.pose.pose.position.y = T_map.translation().y();
    LiDAR_map.pose.pose.position.z = T_map.translation().z();
    Eigen::Quaterniond q;
    q = T_map.rotation();
    LiDAR_map.pose.pose.orientation.x = q.x();
    LiDAR_map.pose.pose.orientation.y = q.y();
    LiDAR_map.pose.pose.orientation.z = q.z();
    LiDAR_map.pose.pose.orientation.w = q.w();
    LiDAR_map.twist.twist.linear.x = icp.increase(0,3)/(FrameTime - LastFrameTime);
    LiDAR_map.twist.twist.linear.y = icp.increase(1,3)/(FrameTime - LastFrameTime);
    LiDAR_map.twist.twist.linear.z = icp.increase(2,3)/(FrameTime - LastFrameTime);

    q.setIdentity();

    if (IMU_q.size() > 800) {
        std::vector<double> IMU_Time_tmp(IMU_Time.end()-800,IMU_Time.end());
        std::vector<Eigen::Quaterniond> IMU_q_tmp(IMU_q.end()-800,IMU_q.end());
        IMU_q = IMU_q_tmp;
        IMU_Time = IMU_Time_tmp;
    }
    LastFrameTime = FrameTime;
}

void LiDAR_matching_lib::InputDownSample() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr vlp_pcd_ds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < vlp_pcd.size(); i = i + 1) {
        pcl::PointXYZI point_xyzi;
        point_xyzi.x = vlp_pcd[i].x;
        point_xyzi.y = vlp_pcd[i].y;
        point_xyzi.z = vlp_pcd[i].z;
        point_xyzi.intensity = vlp_pcd[i].time;
        vlp_pcd_ds_ptr->push_back(point_xyzi);
    }
    sor.setInputCloud(vlp_pcd_ds_ptr);
    sor.setLeafSize(1.5, 1.5, 0.25);
    sor.filter(vlp_ds_pcd);
}

void LiDAR_matching_lib::findRotation(double pointTime, Eigen::Quaterniond &Q)
{
    int index = 0;
    for (int i = 0; i < IMU_Time.size(); ++i) {
        index = i;
        if (pointTime<IMU_Time[i]){
            break;
        }
    }
    if(index == IMU_Time.size()-1){
        Q= IMU_q.back();
    } else{
        double alpha = (pointTime- IMU_Time[index])/(IMU_Time[index+1] - IMU_Time[index]) ;
        Q = IMU_q[index].slerp(alpha , IMU_q[index+1]);
    }
}
//init value from IMU


void LiDAR_matching_lib::genLocalMap() {
    ros::Time processTime;
    processTime = ros::Time::now();
    Eigen::Affine3d curr_pose;
    curr_pose = T_map;

    pcl::PointCloud<pcl::PointXYZI> vlp_pcd_range ;
    std::vector<int> indices_unique;

    for (int i = 0; i < vlp_ds_pcd.size(); i = i + 1) {
        pcl::PointXYZI point_xyzi;
        if(sqrt(vlp_ds_pcd[i].x*vlp_ds_pcd[i].x + vlp_ds_pcd[i].y*vlp_ds_pcd[i].y +vlp_ds_pcd[i].z*vlp_ds_pcd[i].z)<70){
            point_xyzi.x = vlp_ds_pcd[i].x;
            point_xyzi.y = vlp_ds_pcd[i].y;
            point_xyzi.z = vlp_ds_pcd[i].z;
            point_xyzi.intensity = vlp_ds_pcd[i].intensity;
            vlp_pcd_range.push_back(point_xyzi);
        }
    }
    vlp_ds_pcd = vlp_pcd_range;
    std::vector<int> indices; // 存储查询近邻点索引
    Eigen::Vector3d point;
/*     point = Eigen::Vector3d(T_map.translation().x(),T_map.translation().y(),T_map.translation().z());
       pcl::PointNormal temp;
       temp.x = point.x();
       temp.y = point.y();
       temp.z = point.z();
       kdtree->radiusSearch(temp, 55, indices_unique, distances);*/
    for (int i = 0; i < vlp_pcd_range.size(); ++i) {
        std::vector<int> indices; // 存储查询近邻点索引
        Eigen::Vector3d point;
        point = Eigen::Vector3d(vlp_pcd_range[i].x,vlp_pcd_range[i].y,vlp_pcd_range[i].z);
        point =  curr_pose.rotation()*point + curr_pose.translation();
        pcl::PointNormal temp;
        temp.x = point.x();
        temp.y = point.y();
        temp.z = point.z();
        kdtree->nearestKSearch(temp,20,indices,distances);
//        kdtree->radiusSearch(temp, 6, indices, distances);
        for (int j = 0; j < indices.size(); ++j) {
            indices_unique.push_back(indices[j]);
        }
    }
    saveProcessTime("2.1 Kdtree Time: ",processTime.toSec()-ros::Time::now().toSec());
    processTime = ros::Time::now();
//    for (int i = 0; i <indices_last.size() ; ++i) {
//        indices_unique.push_back(indices_last[i]);
//    }
    std::sort(indices_unique.begin(),indices_unique.end());
    indices_unique.erase(std::unique(indices_unique.begin(),indices_unique.end()),indices_unique.end());
    for (int j = 0; j < indices_unique.size(); ++j) {
        LocalMap.push_back(mls_points[indices_unique[j]]);
    }
    indices_last = indices_unique;
    saveProcessTime("2.2 from indices find map Time: ",processTime.toSec()-ros::Time::now().toSec());
    processTime = ros::Time::now();
}

void LiDAR_matching_lib::AccumulateImu() {
    for (int i = 0; i < ImuQueue.size(); ++i) {
            Eigen::Vector3d gyro;
            gyro[0] = -ImuQueue[i].angular_velocity.x;
            gyro[1] =  ImuQueue[i].angular_velocity.y;
            gyro[2] = -ImuQueue[i].angular_velocity.z;
//            gyro[0] = ImuQueue[i].angular_velocity.x;
//            gyro[1] = ImuQueue[i].angular_velocity.y;
//            gyro[2] = ImuQueue[i].angular_velocity.z;
            Eigen::Quaterniond dq;

            Eigen::Vector3d dtheta_half = (gyro_last/4.0 + gyro/4.0) * IMU_period_time ;
            gyro_last = gyro;
            dq.w() = 1;
            dq.x() = dtheta_half.x();
            dq.y() = dtheta_half.y();
            dq.z() = dtheta_half.z();
            if(IMU_q.empty()){
                dq.setIdentity();
                IMU_q.push_back( dq );
            }else{
                dq = IMU_q.back()*dq;
                dq.normalize();
                IMU_q.push_back( dq );
            }
            IMU_Time.push_back(ImuQueue[i].header.stamp.toSec());
    }
    ImuQueue.clear();
    if(IMU_Time.back() >= FrameTime + vlp_pcd.back().time + time_offset){
        imuReady = true;
    } else{
        imuReady = false;
    }
}

void LiDAR_matching_lib::saveProcessTime(std::string name, double value) {
    diagnostic_msgs::KeyValue temp_time;
    temp_time.key = name;
    temp_time.value = std::to_string(value);
    Time_used.values.push_back(temp_time) ;
}


