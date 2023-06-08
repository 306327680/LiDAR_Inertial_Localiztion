//
// Created by echo on 23-6-8.
//


#include <ceres/ceres.h>
#include <vector>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/math/distributions.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>

#include <g2o/stuff/sampler.h>
#include <g2o/core/factory.h>

#include <g2o/stuff/sampler.h>

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
class ICPSimulation {
public:
    ICPSimulation(){};
    ICPSimulation(pcl::PointCloud<pcl::PointXYZI> cloud);
    //~ICPSimulation();

    void start( pcl::PointCloud<pcl::PointXYZI> cloud, Eigen::Matrix4f &predict_pose , Eigen::Matrix4f &result_pose);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_ptr, source_ptr;
    pcl::PointCloud<pcl::PointXYZI> transform_pt;
    Eigen::Matrix<double,6,6> covariance_matrix;
    int max_iterations = 10;
    float max_coresspoind_dis = 2.5;
    float trans_eps =0.001;
private:
    Eigen::Matrix3d information_;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_flann;
};

class ICPErr : public ceres::SizedCostFunction<3, 6> {
public:
    ICPErr(Eigen::Vector3d& pi, Eigen::Vector3d &pj,
           Eigen::Matrix<double, 3, 3> &information);
    virtual ~ICPErr() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;

public:
    Eigen::Vector3d Pi;
    Eigen::Vector3d Pj;
    Eigen::Matrix<double, 3, 3> sqrt_information_;
};

class IMUErr : public ceres::SizedCostFunction<3, 6> {
public:
    IMUErr(Eigen::Vector3d& pi, Eigen::Vector3d &pj,
           Eigen::Matrix<double, 3, 3> &information);
    virtual ~IMUErr() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const;


public:
    Eigen::Vector3d Pi;
    Eigen::Vector3d Pj;
    Eigen::Matrix<double, 3, 3> sqrt_information_;
};

class CERES_EXPORT SE3Parameterization : public ceres::LocalParameterization {
public:
    SE3Parameterization() {}
    virtual ~SE3Parameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 6; }
};
