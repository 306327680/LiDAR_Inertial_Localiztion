//
// Created by echo on 23-5-2.
//

#include "LiDAR_matching_lib.h"

void LiDAR_matching_lib::process() {
    ros::Time processTime;
    pcl::PointCloud<pcl::PointXYZI>::Ptr vlp_pcd_ds_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    LocalMap.clear();
    vlp_pcd_ds_ptr->clear();
    if(InitPoseBool){
        icp.transformation  = T_map.matrix().cast<float>();
        InitPoseBool = false;
        InitPoseCome = true;
    }

    pcl::PointCloud<VLPPoint> vlp_pcd_ds;
    vlp_pcd_ds.clear();
    // 1.Down sample point
    // todo 1.1 Distortion
    // todo 1.2 Transfer to Map frame
    // 2.Get Local Map
    // Todo: 3.Point to Plane ICP
    // 4.

    if(InitPoseCome && newIMU && newLiDAR){
        FrameTime = Point_raw.header.stamp.toSec();
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::fromROSMsg(Point_raw,vlp_pcd);
        //1.1 2
        std::vector<int> indices_unique;
        vlp_ds_pcd.clear();
        processTime = ros::Time::now();
        InputDownSample();
        ImuDistortion(vlp_pcd.front().time,vlp_pcd.back().time);
        Eigen::Affine3d curr_pose;
        setInitParam(curr_pose);
        if(imuReady){
            for (int i = 0; i < vlp_pcd.size(); i = i + 1) {
                pcl::PointXYZI point_xyzi;
                if(sqrt(vlp_pcd[i].x*vlp_pcd[i].x + vlp_pcd[i].y*vlp_pcd[i].y +vlp_pcd[i].z*vlp_pcd[i].z)<50){
                    point_xyzi.x = vlp_pcd[i].x;
                    point_xyzi.y = vlp_pcd[i].y;
                    point_xyzi.z = vlp_pcd[i].z;
                    point_xyzi.intensity = vlp_pcd[i].intensity;

                    vlp_pcd_ds.push_back(vlp_pcd[i]);
                    vlp_pcd_ds_ptr->push_back(point_xyzi);
                }
            }
            sor.setInputCloud(vlp_pcd_ds_ptr);
            sor.setLeafSize(2, 2, 0.5);
            sor.filter(vlp_ds_pcd);
            DebugTime[0] = processTime.toSec()-ros::Time::now().toSec();
            std::cout<<"process Time: "<<processTime.toSec()-ros::Time::now().toSec()<<std::endl;
            processTime = ros::Time::now();
            //todo local map test
            std::vector<int> indices; // 存储查询近邻点索引
            Eigen::Vector3d point;
            point = Eigen::Vector3d(T_map.translation().x(),T_map.translation().y(),T_map.translation().z());

            /*  pcl::PointNormal temp;
              temp.x = point.x();
              temp.y = point.y();
              temp.z = point.z();
              kdtree->radiusSearch(temp, 55, indices_unique, distances);*/
            for (int i = 0; i < vlp_ds_pcd.size(); ++i) {
                std::vector<int> indices; // 存储查询近邻点索引
                Eigen::Vector3d point;
                point = Eigen::Vector3d(vlp_ds_pcd[i].x,vlp_ds_pcd[i].y,vlp_ds_pcd[i].z);
                point =  curr_pose.rotation()*point + curr_pose.translation();
                pcl::PointNormal temp;
                temp.x = point.x();
                temp.y = point.y();
                temp.z = point.z();
                kdtree->radiusSearch(temp, 4, indices, distances);
                for (int j = 0; j < indices.size(); ++j) {
                    indices_unique.push_back(indices[j]);
                }
            }
            std::sort(indices_unique.begin(),indices_unique.end());
            indices_unique.erase(std::unique(indices_unique.begin(),indices_unique.end()),indices_unique.end());

            for (int j = 0; j < indices_unique.size(); ++j) {
                LocalMap.push_back(mls_points[indices_unique[j]]);
            }
            std::cout<<"map Time: "<<processTime.toSec()-ros::Time::now().toSec()<<std::endl;
            DebugTime[1] = processTime.toSec()-ros::Time::now().toSec();
            //3. ICP
            processTime = ros::Time::now();
            registrion(vlp_ds_pcd,LocalMap);
            std::cout<<"icp Time: "<<processTime.toSec()-ros::Time::now().toSec()<<std::endl;
            DebugTime[2] = processTime.toSec()-ros::Time::now().toSec();
            handleMessage();
            imuReady = false;
        }
    }
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
    LiDAR_map_last.header.stamp = ros::Time::now();
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
    for (int i = 0; i < ImuQueue.size(); ++i) {
        if(IMU_Time.back()>=FrameTime+first_point_time){
            imuReady = true;
        } else{
            Eigen::Vector3d gyro;
            gyro[0] = ImuQueue.front().angular_velocity.x;
            gyro[1] = ImuQueue.front().angular_velocity.y;
            gyro[2] = ImuQueue.front().angular_velocity.z;
            Eigen::Quaterniond dq;
            Eigen::Vector3d dtheta_half = (gyro_last+gyro) * IMU_period_time / 2.0;
            gyro_last = gyro;
            dq.w() = 1;
            dq.x() = dtheta_half.x();
            dq.y() = dtheta_half.y();
            dq.z() = dtheta_half.z();
            dq = IMU_q.back()*dq;
            dq.normalize();
            IMU_q.push_back( dq );//todo clear
            IMU_Time.push_back(ImuQueue.front().header.stamp.toSec());
            ImuQueue.pop();
        }
    }
    std::cerr<<(IMU_Time.back()>FrameTime+last_point_time)<<std::endl;
    if(imuReady){
        for (int i = 0; i < vlp_pcd.size(); ++i) {
            Eigen::Quaterniond q;
            findRotation(FrameTime + vlp_pcd[i].time,q);
            Eigen::Vector3d point(vlp_pcd[i].x,vlp_pcd[i].y,vlp_pcd[i].z);
            point =  q.matrix() * point;
            vlp_pcd[i].x = point.x();
            vlp_pcd[i].y = point.y();
            vlp_pcd[i].z = point.z();
        }
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
    Transfer_local_point = icp.normalIcpRegistration(temp,map_T_last);
    T_map = T_map.matrix() * icp.pcl_plane_plane_icp->getFinalTransformation().cast<double>();
    pcl::transformPointCloud(Transfer_local_point, local_map, T_map.matrix().cast<float>());
    Transfer_local_point = local_map;
}

void LiDAR_matching_lib::handleMessage() {
    //4.pub
    pcl::PCLPointCloud2 pcl_frame;
    pcl::toPCLPointCloud2(Transfer_local_point, pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, LiDAR_Distort);
    LiDAR_Distort.header = Point_raw.header;
    LiDAR_Distort.header.frame_id = "/map";

    pcl::toPCLPointCloud2(LocalMap,pcl_frame);
    pcl_conversions::fromPCL(pcl_frame, LocalMapPC2);
    LocalMapPC2.header.stamp = Point_raw.header.stamp;
    LocalMapPC2.header.frame_id = "/map";


    LiDAR_map.header.stamp = Point_raw.header.stamp;
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

    //reset IMU
    IMU_q.clear();
    IMU_Time.clear();
    IMU_q.push_back( q );
    IMU_Time.push_back( Point_raw.header.stamp.toSec());
}

void LiDAR_matching_lib::InputDownSample() {

}

void LiDAR_matching_lib::findRotation(double pointTime, Eigen::Quaterniond &Q)
{
    int index = 0;
    for (int i = 0; i < IMU_Time.size(); ++i) {
        index = i;
        if (pointTime>IMU_Time[i]){
            break;
        }
    }
    if(index == IMU_Time.size()-1){
        Q= IMU_q.back();
    } else{
        double alpha = (pointTime- IMU_Time[index])/(IMU_Time[index] -  IMU_Time[index+1]) ;
        Q = IMU_q[index].slerp(alpha , IMU_q[index+1]);
    }
}
//init value from IMU
void LiDAR_matching_lib::setInitParam(Eigen::Affine3d &pose) {
    pose = T_map;// todo predict the cloud map pose// <<later us imu!!!!>>
    LiDAR_map_last.pose.pose.position.x = T_map.translation().x();
    LiDAR_map_last.pose.pose.position.y = T_map.translation().y();
    LiDAR_map_last.pose.pose.position.z = T_map.translation().z();
}


