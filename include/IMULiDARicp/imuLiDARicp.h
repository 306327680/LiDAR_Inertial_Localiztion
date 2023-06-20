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
    ICPSimulation(pcl::PointCloud<pcl::PointNormal> cloud);
    //~ICPSimulation();

    void start( pcl::PointCloud<pcl::PointXYZI> cloud, Eigen::Matrix4f &predict_pose , Eigen::Matrix4f &result_pose);
    void pointPlane( pcl::PointCloud<pcl::PointXYZI> cloud, Eigen::Matrix4f &predict_pose , Eigen::Matrix4f &result_pose);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_ptr, source_ptr;
    pcl::PointCloud<pcl::PointNormal>::Ptr target_normal_ptr;
    pcl::PointCloud<pcl::PointXYZI> transform_pt;
    pcl::PointCloud<pcl::PointNormal> transform_normal_pt;
    Eigen::Matrix<double,6,6> covariance_matrix;
    int max_iterations = 10;
    float max_coresspoind_dis = 2.5;
    float trans_eps =0.001;
private:
    Eigen::Matrix3d information_;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_flann;
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree_normal_flann;
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

struct LidarPlaneFactor
{
    LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d target_point_,
                     Eigen::Vector3d ljm_norm_, double s_)
            : curr_point(curr_point_), target_point(target_point_), ljm_norm(ljm_norm_),s(s_)
    {
        ljm_norm.normalize();
    }

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {

        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpj{T(target_point.x()), T(target_point.y()), T(target_point.z())};
        Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;

        residual[0] = (lp - lpj).dot(ljm);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d target_point_,
                                       const Eigen::Vector3d ljm_norm_, const double s_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarPlaneFactor, 1, 4, 3>(
                new LidarPlaneFactor(curr_point_, target_point_, ljm_norm_,  s_)));
    }

    Eigen::Vector3d curr_point, target_point, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
    double s;
};
