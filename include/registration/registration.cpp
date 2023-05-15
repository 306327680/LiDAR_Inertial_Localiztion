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
	icp->setMaximumIterations(100);
	icp->setMaxCorrespondenceDistance(3);
	icp->setTransformationEpsilon(0.001);
	icp->setEuclideanFitnessEpsilon(0.001);
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

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZI> tfed;
	pcl::copyPointCloud(target,*cloud_target_normals);

	addNormal(source, cloud_source_normals);
	local_map_with_normal = *cloud_target_normals;

    pcl_plane_plane_icp->setInputSource(cloud_source_normals);
    pcl_plane_plane_icp->setInputTarget(cloud_target_normals);
    pcl_plane_plane_icp->align(*cloud_source_normals);

    transformation =  pcl_plane_plane_icp->getFinalTransformation();//上次结果(结果加预测)

    increase =  pcl_plane_plane_icp->getFinalTransformation();
	pcl::transformPointCloud(*source, tfed, transformation.matrix());
	return tfed;
}

void registration::icp(pcl::PointCloud<pcl::PointXYZI>::Ptr source, pcl::PointCloud<pcl::PointNormal> target) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4f T;
    for (int i = 0; i < target.size(); ++i) {
        pcl::PointXYZI temp;
        temp.x = target[i].x;
        temp.y = target[i].y;
        temp.z = target[i].z;
        target_->push_back(temp);

    }
//    ICP.setMaximumIterations(80);
//    ICP.setMaxCorrespondenceDistance(2.5);
//    ICP.setTransformationEpsilon(0.0000001);
//    ICP.setEuclideanFitnessEpsilon(0.00001);
//    ICP.setInputSource(source);
//    ICP.setInputTarget(target_);
//
//    ICP.align(*source_);
//    transformation =  ICP.getFinalTransformation();//上次结果(结果加预测)
//    increase =  ICP.getFinalTransformation();

    TESTICP::opt_ICP_CERES ICP_ceres;
    ICP_ceres.max_iterations = 15;
    ICP_ceres.max_coresspoind_dis = 1.5;
    ICP_ceres.trans_eps = 0.0001;
    ICP_ceres.setTargetCloud(target_);
    ICP_ceres.scanMatch(source, Eigen::Matrix4f::Identity(), transformed_source, T);
    covariance_matrix = ICP_ceres.covariance_matrix;
    transformation = T;
    increase =  T;
//    std::cout << T << std::endl;
}
