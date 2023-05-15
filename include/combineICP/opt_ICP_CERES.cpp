#include "opt_ICP_CERES.h"
#include "combineICP/lidarCeres.h"

namespace TESTICP
{
    opt_ICP_CERES::opt_ICP_CERES( )
        : kdtree_flann(new pcl::KdTreeFLANN<PointType>)
    {

//        max_iterations = node["max_iter"].as<int>();
//        max_coresspoind_dis = node["max_corr_dist"].as<float>();
//        trans_eps = node["trans_eps"].as<float>();
//        euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    }

    opt_ICP_CERES::~opt_ICP_CERES()
    {
    }

    bool opt_ICP_CERES::setTargetCloud(const CLOUD_PTR &target)
    {
        target_ptr = target;
        kdtree_flann->setInputCloud(target);
    }
//todo 1.add IMU measurement 2. covariance
    bool opt_ICP_CERES::scanMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                                  CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose)
    {
        source_ptr = source;
        CLOUD_PTR transform_cloud(new CLOUD());
        Eigen::Matrix4d T = predict_pose.cast<double>();
        Eigen::Matrix4d T_last = Eigen::Matrix4d::Identity();
        q_w_curr = Eigen::Quaterniond(T.block<3, 3>(0, 0));
        RPY_curr = q_w_curr.toRotationMatrix().eulerAngles(0, 1, 2);
        t_w_curr = T.block<3, 1>(0, 3);
        covariance_matrix.setZero();
        double covariance_xx[7 * 7];
        for (int i = 0; i < max_iterations; ++i)
        {
            pcl::transformPointCloud(*source_ptr, *transform_cloud, T);
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new test_ceres::PoseSE3Parameterization());

            for (int j = 0; j < transform_cloud->size(); ++j)
            {
                const PointType &origin_pt = source_ptr->points[j];
                const PointType &transform_pt = transform_cloud->at(j);
                std::vector<float> res_dis;
                std::vector<int> indices;
                kdtree_flann->nearestKSearch(transform_pt, 1, indices, res_dis);
                if (res_dis.front() > max_coresspoind_dis)
                    continue;
                Eigen::Vector3d nearest_pt = Eigen::Vector3d(target_ptr->at(indices.front()).x,
                                                             target_ptr->at(indices.front()).y,
                                                             target_ptr->at(indices.front()).z);
                Eigen::Vector3d origin_eigen(origin_pt.x, origin_pt.y, origin_pt.z);
                ceres::CostFunction *cost_function = new test_ceres::EdgeAnalyticCostFuntion(origin_eigen, nearest_pt);
                problem.AddResidualBlock(cost_function, loss_function, parameters);
            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 5;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.num_threads = 4;
            options.gradient_check_relative_precision = 0.0001;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            T.setIdentity();
            T.block<3, 1>(0, 3) = t_w_curr;
            T.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();
            double diff = sqrt((T*T_last.inverse())(0,3)*(T*T_last.inverse())(0,3) +
                    (T*T_last.inverse())(1,3)*(T*T_last.inverse())(1,3) +
                    (T*T_last.inverse())(2,3)*(T*T_last.inverse())(2,3));
            if(diff<trans_eps || i == max_iterations-1){
                std::cout<<"inter times: "<<i<<" error: "<<diff<<std::endl;
                //cov
                ceres::Covariance::Options options_c;
                ceres::Covariance covariance(options_c);
                std::vector<std::pair<const double*, const double*> > covariance_blocks;
                covariance_blocks.push_back(std::make_pair(parameters,parameters));
                CHECK(covariance.Compute(covariance_blocks, &problem));
                covariance.GetCovarianceBlock(parameters, parameters, covariance_xx);
                //end conv
                break;
            }else{
                T_last = T;
            }
        }
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                covariance_matrix(j,k) = covariance_xx[j*7+k];
            }
        }
        final_pose = T.cast<float>();
        result_pose = T.cast<float>();
        pcl::transformPointCloud(*source_ptr, *transformed_source_ptr, result_pose);
        return true;
    }

    float opt_ICP_CERES::getFitnessScore()
    {
        float max_range = std::numeric_limits<float>::max();
        float score = 0.f;

        CLOUD_PTR transform_cloud(new CLOUD());
        pcl::transformPointCloud(*source_ptr, *transform_cloud, final_pose);
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);

        int nr = 0;

        for (size_t i = 0; i < transform_cloud->size(); ++i)
        {
            kdtree_flann->nearestKSearch(transform_cloud->points[i], 1, nn_indices, nn_dists);
            if (nn_dists.front() <= max_range)
            {
                score += nn_dists.front();
                nr++;
            }
        }
        if (nr > 0)
            return score / static_cast<float>(nr);
        else
            return (std::numeric_limits<float>::max());
    }
}