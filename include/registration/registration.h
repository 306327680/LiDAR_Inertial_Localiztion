//
// Created by echo on 2019/11/26.
//

#ifndef PCD_COMPARE_REGISTRATION_H
#define PCD_COMPARE_REGISTRATION_H
/*#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"*/
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d_omp.h>
#include "pointType/pointTypes.h"
#include <pcl/surface/mls.h>

#include "IMULiDARicp/imuLiDARicp.h"
class registration {

public:
	registration(){reset();};//初始化ptr用的
	void reset(){pcl_plane_plane_icp.reset(new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>);}
	//pcl 配准部分
	void addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals);
	//scan-scan 参数
	void SetNormalICP();
	//scan-map参数
    pcl::PointCloud<pcl::PointXYZI> normalIcpRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
                                                          pcl::PointCloud<pcl::PointNormal> target);
    void icp(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
             pcl::PointCloud<pcl::PointNormal> target);
	Eigen::Matrix4f transform_frame_to_frame = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(); //全局位姿
    Eigen::Matrix4f initialPose = Eigen::Matrix4f::Identity();;
	Eigen::Matrix4f icp_init = Eigen::Matrix4f::Identity();//icp的初值
	Eigen::Matrix4f increase = Eigen::Matrix4f::Identity();//两次icp的结果
    Eigen::Matrix<double,6,6> covariance_matrix;
    //for imu
    Eigen::Vector3d IMU_p_latest;
	//tools ReOrthogonalization 防止累计误差
	Eigen::Isometry3d  ReOrthogonalization(Eigen::Isometry3d input);
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr pcl_plane_plane_icp;
    pcl::IterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> ICP;
	//可视化normal
	pcl::PointCloud<pcl::PointXYZINormal> local_map_with_normal;
};


#endif //PCD_COMPARE_REGISTRATION_H
