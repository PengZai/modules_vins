#include "bundle_adjustment.h"





namespace modules_vins
{


bool bundleAdjustment(Sophus::SE3<double> &Transformation, std::vector<Eigen::Vector3d> &pts3d, std::vector<Eigen::Vector2d> &observation_pts2d, const Eigen::Matrix3d &K, const Eigen::VectorXd &distortion_coeffs){


    Eigen::Vector3d rotation_vec = Transformation.so3().log();
    Eigen::Vector3d translation = Transformation.translation();


    double pose[6];
    for (int i = 0; i < 3; ++i) {
        pose[i] = rotation_vec[i];
        pose[i + 3] = translation[i];
    }



    ceres::Problem problem;
    for (int obs_idx=0; obs_idx < observation_pts2d.size(); obs_idx++) {
        Eigen::Vector2d pt2d = observation_pts2d.at(obs_idx);
        // std::cout << "pt2f : " << pt2f << " pt3f : " << point3d_for_ceres[obs_idx](0) << " , " << point3d_for_ceres[obs_idx](1) << " , "  << point3d_for_ceres[obs_idx](2) << std::endl;
        ceres::CostFunction* cost_function =
            ReprojectionError::Create(pt2d(0), pt2d(1), K, distortion_coeffs);
    
        problem.AddResidualBlock(cost_function, nullptr, pose, pts3d[obs_idx].data());
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    // options.num_linear_solver_threads = 1;
    // options.max_num_iterations = 50;
    // options.num_threads = 1;
    // options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);


    double final_cost = summary.final_cost;

    if(final_cost > options.function_tolerance*1e4){
        return false;
    }

    Transformation = Sophus::SE3d(
        Sophus::SO3d::exp(Eigen::Vector3d(pose[0], pose[1], pose[2])),  // convert angle-axis to SO(3)
        Eigen::Vector3d(pose[3], pose[4], pose[5]));                     // set translation
    

    return true;

}
    
    
} // namespace modules_vins





