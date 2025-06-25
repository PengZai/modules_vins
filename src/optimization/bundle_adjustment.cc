#include "bundle_adjustment.h"





namespace modules_vins
{


bool bundleAdjustmentPoseOnlyCeres(
  std::vector<Eigen::Vector3d> &points_3d,
  std::vector<Eigen::Vector2d> &points_2d,
  const Eigen::Matrix<double, 3, 3> &K,
  Sophus::SE3d &pose
) {
  
  Eigen::Matrix<double, 1, 6> se3_vec = pose.log().transpose();
  ceres::LossFunction* loss_function =
                new ceres::HuberLoss(1.0);
  ceres::Problem problem;
  
  for(int idx=0;idx<points_2d.size();idx++){


    ceres::CostFunction *cost_function = new reprojectionCostFunctionForPoseOnly(points_3d[idx], points_2d[idx], K);
    problem.AddResidualBlock(cost_function, loss_function, se3_vec.data());
  }
  
  

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 50;
  // options.gradient_tolerance = 1e-10;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";


  pose = Sophus::SE3d::exp(se3_vec);
  std::cout << "only pose estimation: \n" << pose.matrix() << std::endl;

  return true;

}
    
} // namespace modules_vins





