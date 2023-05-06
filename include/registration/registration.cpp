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


pcl::PointCloud<pcl::PointXYZI> registration::normalIcpRegistration(pcl::PointCloud<VLPPoint>::Ptr source,
													  pcl::PointCloud<pcl::PointNormal> target){
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals_temp(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZI> tfed;

	Eigen::Matrix4f transformation_local = Eigen::Matrix4f::Identity(); //全局tf
	Eigen::Matrix4f icp_init_local = Eigen::Matrix4f::Identity();//初值
	//隔断一下
	pcl::PointCloud<pcl::PointXYZI>::Ptr source1(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(target,*cloud_target_normals);

	for (auto & i : *source) {
		pcl::PointXYZI tmp;
		tmp.x = i.x;
		tmp.y = i.y;
		tmp.z = i.z;
		tmp.intensity = i.intensity;
		source1->push_back(tmp);
	}
	addNormal(source1, cloud_source_normals);

	local_map_with_normal = *cloud_target_normals;

    *cloud_source_normals_temp = *cloud_source_normals;
    //0. 当前预测量 = 上次位姿态*增量
    icp_init = transformation ;

    //去除累计误差
    icp_init = ReOrthogonalization(Eigen::Isometry3d(icp_init.matrix().cast<double>())).matrix().cast<float>();
    //1.转换点云 给一个初值
    pcl::transformPointCloud(*cloud_source_normals_temp, *cloud_source_normals, icp_init.matrix());

    pcl_plane_plane_icp->setInputSource(cloud_source_normals);
    pcl_plane_plane_icp->setInputTarget(cloud_target_normals);
    pcl_plane_plane_icp->align(*cloud_source_normals);
    //2.当前的transform 全局准确
    transformation = icp_init * pcl_plane_plane_icp->getFinalTransformation();//上次结果(结果加预测)
    //计算不带increase的increase 1上次位姿 * 预测 * 预测的调整 是错的 应该是 :
    //实际增量 = 上次增量* icp算出的增量误差
    //上面那个也不对
    //increase = transformation * increase * pcl_plane_plane_icp->getFinalTransformation();
    increase = increase * pcl_plane_plane_icp->getFinalTransformation();
	pcl::transformPointCloud(*source1, tfed, transformation.matrix());
	//变化量
	return tfed;
}