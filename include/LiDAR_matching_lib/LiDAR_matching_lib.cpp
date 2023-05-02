//
// Created by echo on 23-5-2.
//

#include "LiDAR_matching_lib.h"

void LiDAR_matching_lib::process() {
    LocalMap.clear();
    pcl::PointCloud<VLPPoint> vlp_pcd_tmp;
    pcl::PCLPointCloud2 pcl_frame;
    vlp_pcd_tmp.clear();
    // 1.Down sample point
    // todo 1.1 Distortion
    // todo 1.2 Transfer to Map frame
    // 2.Get Local Map
    // Todo: 3.Point to Plane ICP
    // 4.
    if(InitPoseBool && newIMU && newLiDAR){
        FrameTime = Point_raw.header.stamp.toSec();
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::fromROSMsg(Point_raw,vlp_pcd);
        ImuDistortion(vlp_pcd.front().time,vlp_pcd.back().time); //todo finish later
        for (int i = 0; i < vlp_pcd.size(); i = i + 81) {
            vlp_pcd_tmp.push_back(vlp_pcd[i]);
            pcl::PointNormal temp;
            temp.x = vlp_pcd[i].x;
            temp.y = vlp_pcd[i].y;
            temp.z = vlp_pcd[i].z;
            kdtree->nearestKSearch(temp, 10, indices, distances);
            for (int j = 0; j < indices.size(); ++j) {
                LocalMap.push_back(mls_points[indices[j]]);
            }
        }
        pcl::toPCLPointCloud2(vlp_pcd_tmp, pcl_frame);
        pcl_conversions::fromPCL(pcl_frame, LiDAR_Distort);
        LiDAR_Distort.header = Point_raw.header;

        pcl::toPCLPointCloud2(LocalMap,pcl_frame);
        pcl_conversions::fromPCL(pcl_frame, LocalMapPC2);
        LocalMapPC2.header.stamp = Point_raw.header.stamp;
        LocalMapPC2.header.frame_id = "/map";
    }
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
}
//todo later
void LiDAR_matching_lib::ImuDistortion(double first_point_time,double last_point_time) {
    std::vector<sensor_msgs::Imu> sync_imu;
    for (int i = 0; i < ImuQueue.size(); ++i) {
        if (ImuQueue.front().header.stamp.toSec()<FrameTime+first_point_time){
            ImuQueue.pop();
        }
        if(ImuQueue.front().header.stamp.toSec()>=FrameTime+first_point_time && ImuQueue.front().header.stamp.toSec()<=FrameTime+last_point_time){
            sync_imu.push_back(ImuQueue.front());
            ImuQueue.pop();
        }
    }
    //std::cerr<<FrameTime<<" ImuQueue.size(): "<<ImuQueue.size()<<" sync_imu: "<<sync_imu.size()<<std::endl;
}
