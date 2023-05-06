//
// Created by echo on 2019/11/26.
//

#include "registration.h"

//给一个点云添加normal
void registration::addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
							 pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud,*cloud_source_normals);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZ>);
	searchTree->setInputCloud(cloud_source_normals);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimator_pa;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
	bool omp = true;
	if(omp){
		normalEstimator_pa.setInputCloud(cloud_source_normals);
		normalEstimator_pa.setSearchMethod(searchTree);
		//normalEstimator_pa.setRadiusSearch(0.05);
		normalEstimator_pa.setKSearch(6);//20
		normalEstimator_pa.compute(*normals);
		pcl::concatenateFields(*cloud_source_normals, *normals, *cloud_with_normals);
	}else{
		normalEstimator.setInputCloud(cloud_source_normals);
		normalEstimator.setSearchMethod(searchTree);
		//normalEstimator.setRadiusSearch(0.05);
		normalEstimator.setKSearch(20);
		normalEstimator.compute(*normals);
		pcl::concatenateFields(*cloud_source_normals, *normals, *cloud_with_normals);
	}

}

void registration::SetNormalICP() {
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(
			new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	icp->setMaximumIterations(10);
	icp->setMaxCorrespondenceDistance(20);
	icp->setTransformationEpsilon(0.01);
	icp->setEuclideanFitnessEpsilon(0.01);
	this->pcl_plane_plane_icp = icp;
	
}

//旋转normalize
Eigen::Isometry3d registration::ReOrthogonalization(Eigen::Isometry3d input)  {
 
		Eigen::Isometry3d result = Eigen::Isometry3d::Identity() ;
		Eigen::Matrix3d rotation(input.rotation());
		Eigen::Matrix4d se3;
		Eigen::Matrix4d diff;
		Eigen::Quaterniond Quat;
		se3 = input.matrix();
		
		Quat = rotation;
		Quat.normalize();
		//r
		result.setIdentity();
		result.rotate(Quat);
 		//t
		result(0,3) = se3(0,3);
		result(1,3) = se3(1,3);
		result(2,3) = se3(2,3);
	 
		return result;
 
}


pcl::PointCloud<pcl::PointXYZI> registration::normalIcpRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
													  pcl::PointCloud<pcl::PointNormal> target){
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals_temp(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZI> tfed;
	pcl::copyPointCloud(target,*cloud_target_normals);

	addNormal(source, cloud_source_normals);
	local_map_with_normal = *cloud_target_normals;
    *cloud_source_normals_temp = *cloud_source_normals;
    //0. 当前预测量 = 上次位姿态*增量
    icp_init = transformation ;
    icp_init = ReOrthogonalization(Eigen::Isometry3d(icp_init.matrix().cast<double>())).matrix().cast<float>();
    //1.转换点云 给一个初值
    pcl::transformPointCloud(*cloud_source_normals_temp, *cloud_source_normals, icp_init.matrix());

    pcl_plane_plane_icp->setInputSource(cloud_source_normals);
    pcl_plane_plane_icp->setInputTarget(cloud_target_normals);
    pcl_plane_plane_icp->align(*cloud_source_normals);

    transformation = icp_init * pcl_plane_plane_icp->getFinalTransformation();//上次结果(结果加预测)
    increase = increase * pcl_plane_plane_icp->getFinalTransformation();
	pcl::transformPointCloud(*source, tfed, transformation.matrix());
	return tfed;
}