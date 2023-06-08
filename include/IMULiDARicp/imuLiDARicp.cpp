//
// Created by echo on 23-6-8.
//

#include "imuLiDARicp.h"

bool ICPErr::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(parameters[0]);
    Sophus::SE3d T = Sophus::SE3d::exp(lie);

    auto Pj_ = T * Pi;
    Eigen::Vector3d err = Pj - Pj_;

    residuals[0] = err(0);
    residuals[1] = err(1);
    residuals[2] = err(2);

    Eigen::Matrix<double, 3, 6> Jac = Eigen::Matrix<double, 3, 6>::Zero();
    Jac.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    Jac.block<3, 3>(0, 3) = Sophus::SO3d::hat(Pj_);
    int k = 0;
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 6; ++j) {
            if(k >= 18)
                return false;
            if(jacobians) {
                if(jacobians[0])
                    jacobians[0][k] = Jac(i, j);
            }
            k++;
        }
    }
    return true;
}

ICPErr::ICPErr(Eigen::Vector3d& pi, Eigen::Vector3d &pj,
               Eigen::Matrix<double, 3, 3>& information) :  Pi(pi), Pj(pj) {

    //printf("index = %d\n", index++);
    Eigen::LLT<Eigen::Matrix<double, 3, 3>> llt(information);
    sqrt_information_ = llt.matrixL();
}



bool SE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
    return true;
}

bool SE3Parameterization::Plus(const double* x,
                               const double* delta,
                               double* x_plus_delta) const {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);

    Sophus::SE3d T = Sophus::SE3d::exp(lie);
    Sophus::SE3d delta_T = Sophus::SE3d::exp(delta_lie);
    Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (delta_T * T).log();

    for(int i = 0; i < 6; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);

    return true;

}

/* ############################################################################################
 * ############################################################################################
 */


ICPSimulation::ICPSimulation( pcl::PointCloud<pcl::PointXYZI> cloud)  : kdtree_flann(new pcl::KdTreeFLANN<pcl::PointXYZI>) ,
                                                                        target_ptr( new pcl::PointCloud<pcl::PointXYZI> ) ,source_ptr ( new pcl::PointCloud<pcl::PointXYZI> ){
    *target_ptr = cloud;
    kdtree_flann.setInputCloud(target_ptr);
}

void ICPSimulation::start( pcl::PointCloud<pcl::PointXYZI> cloud, Eigen::Matrix4f &predict_pose , Eigen::Matrix4f &result_pose) {
    Eigen::Matrix4f T_last = Eigen::Matrix4f::Identity();
    double covariance_xx[6 * 6];
    double se3[6];
    Eigen::Vector3d IMUtrans = Eigen::Vector3d (0,0,0);
    Eigen::Vector3d centerpoint = Eigen::Vector3d (0,0,0);
    memset(se3, 0, 6 * sizeof(double));
    for (int i = 0; i < max_iterations; ++i) {
        pcl::transformPointCloud(cloud, transform_pt, T_last);
        ceres::Problem problem;
        for (size_t i = 0; i < transform_pt.size(); ++i) {
            std::vector<float> res_dis;
            std::vector<int> indices;
            kdtree_flann.nearestKSearch(transform_pt[i], 1, indices, res_dis);
            if (res_dis.front() > max_coresspoind_dis)
                continue;
            Eigen::Vector3d nearest_pt = Eigen::Vector3d(target_ptr->at(indices.front()).x,
                                                         target_ptr->at(indices.front()).y,
                                                         target_ptr->at(indices.front()).z);
            Eigen::Vector3d origin_eigen(cloud[i].x, cloud[i].y, cloud[i].z);
            ceres::CostFunction *costFun = new ICPErr(origin_eigen, nearest_pt, information_);
            problem.AddResidualBlock(costFun, new ceres::HuberLoss(0.5), se3);
        }
        ceres::CostFunction *costFun1 = new IMUErr(IMUtrans, centerpoint, information_);
        problem.AddResidualBlock(costFun1, new ceres::TrivialLoss, se3);
        problem.SetParameterization(se3, new SE3Parameterization());

        ceres::Solver::Options options;
        options.minimizer_type = ceres::TRUST_REGION;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.minimizer_progress_to_stdout = false;
        //options.num_threads = 4;
        options.dogleg_type = ceres::SUBSPACE_DOGLEG;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        Eigen::Map<Eigen::Matrix<double, 6, 1> > se3lie(se3);
        result_pose.matrix() = Sophus::SE3d::exp(se3lie).matrix().cast<float>();

        double diff = sqrt((result_pose*T_last.inverse())(0,3)*(result_pose*T_last.inverse())(0,3) +
                           (result_pose*T_last.inverse())(1,3)*(result_pose*T_last.inverse())(1,3) +
                           (result_pose*T_last.inverse())(2,3)*(result_pose*T_last.inverse())(2,3));
        if(diff<trans_eps || i == max_iterations-1){
//            std::cout<<"inter times: "<<i<<" error: "<<diff<<std::endl;
            //cov
            ceres::Covariance::Options options_c;
            ceres::Covariance covariance(options_c);
            std::vector<std::pair<const double*, const double*> > covariance_blocks;
            covariance_blocks.push_back(std::make_pair(se3,se3));
            CHECK(covariance.Compute(covariance_blocks, &problem));
            covariance.GetCovarianceBlock(se3, se3, covariance_xx);
            for (int j = 0; j < 6; ++j) {
                for (int k = 0; k < 6; ++k) {
                    covariance_matrix(j,k) = covariance_xx[j*6+k];
                }
            }
            break;
        }else{
            T_last = result_pose;
        }
    }
}

IMUErr::IMUErr(Eigen::Vector3d& pi, Eigen::Vector3d &pj,
               Eigen::Matrix<double, 3, 3>& information) :  Pi(pi), Pj(pj) {

    //printf("index = %d\n", index++);
    Eigen::LLT<Eigen::Matrix<double, 3, 3>> llt(information);
    sqrt_information_ = llt.matrixL();
}

bool IMUErr::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {

    Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(parameters[0]);
    Sophus::SE3d T = Sophus::SE3d::exp(lie);

    auto Pj_ = T * Pi;
    Eigen::Vector3d err = Pj - Pj_;

    residuals[0] = err(0);
    residuals[1] = err(1);
    residuals[2] = err(2);

    Eigen::Matrix<double, 3, 6> Jac = Eigen::Matrix<double, 3, 6>::Zero();
    Jac.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    Jac.block<3, 3>(0, 3) = Sophus::SO3d::hat(Pj_);
    int k = 0;
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 6; ++j) {
            if(k >= 18)
                return false;
            if(jacobians) {
                if(jacobians[0])
                    jacobians[0][k] = Jac(i, j);
            }
            k++;
        }
    }
    return true;
}
