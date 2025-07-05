#include "bundle_adjustment.h"





namespace modules_vins
{


// bool bundleAdjustmentPoseOnlyCeres(
//   std::vector<Eigen::Vector3d> &points_3d,
//   std::vector<Eigen::Vector2d> &points_2d,
//   Sophus::SE3d &pose
// ) {
  
//   Eigen::Matrix<double, 1, 6> se3_vec = pose.log().transpose();
//   ceres::LossFunction* loss_function =
//                 new ceres::HuberLoss(1.0);
//   ceres::Problem problem;
  
//   for(int idx=0;idx<points_2d.size();idx++){


//     ceres::CostFunction *cost_function = new reprojectionCostFunctionForPoseOnly(points_3d[idx], points_2d[idx]);
//     problem.AddResidualBlock(cost_function, loss_function, se3_vec.data());
//   }
  
  

//   // Run the solver!
//   ceres::Solver::Options options;
//   options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
//   options.minimizer_progress_to_stdout = true;
//   options.max_num_iterations = 50;
//   // options.gradient_tolerance = 1e-10;
//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);
//   std::cout << summary.FullReport() << "\n";


//   pose = Sophus::SE3d::exp(se3_vec);
//   std::cout << "only pose estimation: \n" << pose.matrix() << std::endl;

//   return true;

// }


// int bundleAdjustmentPoseOnlyCeres(
//   std::vector<Eigen::Vector3d> &points_3d,
//   std::vector<Eigen::Vector2d> &points_2d,
//   const Eigen::Matrix<double, 3, 3> &K,
//   const Eigen::Matrix4d &Tc0b,
//   Sophus::SE3d &pose
// ) {
  
//   Eigen::Matrix<double, 1, 6> se3_vec = pose.log().transpose();



//   std::vector<bool> is_outlier_cost_functions;
//   for(int idx=0;idx<points_2d.size();idx++){
//       is_outlier_cost_functions.emplace_back(false);
//   }

//   for(int iter = 0; iter < 3; iter++){

//     ceres::Problem problem;
//     ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
//     // Run the solver!
//     ceres::Solver::Options options;
//     options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
//     options.minimizer_progress_to_stdout = true;
//     options.max_num_iterations = 20;
//     // options.logging_type = ceres::SILENT;
//     // options.gradient_tolerance = 1e-10;
//     ceres::Solver::Summary summary;
//     std::vector<ceres::ResidualBlockId> residual_ids;

//     for(int idx=0; idx < is_outlier_cost_functions.size(); idx++){

//       bool is_outlier = is_outlier_cost_functions[idx];
//       if(is_outlier == false){
//         ceres::ResidualBlockId residual_id;
//         ceres::CostFunction* cost_function = new reprojectionCostFunctionForPoseOnlyWithK(points_3d[idx], points_2d[idx], K, Tc0b);
//         // if(iter < finer_adjustment_iter){
//         //   residual_id = problem.AddResidualBlock(cost_function, loss_function, se3_vec.data());
//         // }
//         // else{
//         //   residual_id = problem.AddResidualBlock(cost_function, nullptr, se3_vec.data());
//         // }
//         residual_id = problem.AddResidualBlock(cost_function, loss_function, se3_vec.data());
//         residual_ids.emplace_back(residual_id);
//       }
//       else{
//         residual_ids.emplace_back(nullptr);
//       }

//     }

//     ceres::Solve(options, &problem, &summary);
//     // std::cout << summary.FullReport() << "\n";

//     ceres::Problem::EvaluateOptions eval_options;
//     eval_options.apply_loss_function = false;
//     std::vector<double> costs, costs_for_threshold;

//     for (size_t i = 0; i < residual_ids.size(); i++) {

//       if(residual_ids[i]){

//         const ceres::ResidualBlockId& residual_id = residual_ids[i];

//         eval_options.residual_blocks = {residual_id};  // vector of ResidualBlockId

//         std::vector<double> residual_errors;
//         double cost = 0.0;

//         problem.Evaluate(eval_options, &cost, &residual_errors, nullptr, nullptr);
//         costs.emplace_back(cost);
//         costs_for_threshold.emplace_back(cost);
//       }      
//       else{
//         costs.emplace_back(-1);
//       }

//     }

//     std::sort(costs_for_threshold.begin(), costs_for_threshold.end());
//     size_t  max_cost_idx = static_cast<size_t>(0.98 * costs_for_threshold.size());
//     size_t  threshold_idx = static_cast<size_t>(0.90 * costs_for_threshold.size());
//     double max_cost = costs_for_threshold[max_cost_idx];
//     double cost_threshold = costs_for_threshold[std::min(threshold_idx, costs_for_threshold.size() - 1)];
//     if(max_cost < 4){
//       break;
//     }

//     for(size_t i = 0; i < residual_ids.size(); i++){
//       if(residual_ids[i]){
//         double cost = costs[i];
//         if(cost > cost_threshold){
//           is_outlier_cost_functions[i] = true;
//         }
//       }
//     }

//   }

//   int num_inlier = 0;
//   int num_outlier = 0;

//   for(int i=0;i<is_outlier_cost_functions.size();i++){
//     if(is_outlier_cost_functions[i] == false){
//       num_inlier++;
//     } 
//     else{
//       num_outlier++;
//     }
//   }
  


//   pose = Sophus::SE3d::exp(se3_vec);
//   LOG(INFO) << GREEN << "only pose estimation: \n" << pose.matrix() << "\n inlier_num : " << num_inlier << "/" << num_outlier << RESET;


//   return num_inlier;

// }


int bundleAdjustmentPoseOnlyCeres(
  std::vector<Eigen::Vector3d> &points_3d,
  std::vector<Eigen::Vector2d> &points_2d,
  const Eigen::Matrix<double, 3, 3> &K,
  Sophus::SE3d &Tbw,
  Sophus::SE3d &Tcb
) {
  
  Eigen::Matrix<double, 1, 6> se3_vec = Tbw.log().transpose();



  std::vector<bool> is_outlier_cost_functions;
  for(int idx=0;idx<points_2d.size();idx++){
      is_outlier_cost_functions.emplace_back(false);
  }

  for(int iter = 0; iter < 3; iter++){

    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 20;
    // options.logging_type = ceres::SILENT;
    // options.gradient_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    std::vector<ceres::ResidualBlockId> residual_ids;

    for(int idx=0; idx < is_outlier_cost_functions.size(); idx++){

      bool is_outlier = is_outlier_cost_functions[idx];
      if(is_outlier == false){
        ceres::ResidualBlockId residual_id;
        ceres::CostFunction* cost_function = new reprojectionCostFunctionForPoseOnlyWithK(points_3d[idx], points_2d[idx], K, Tcb);
        // if(iter < finer_adjustment_iter){
        //   residual_id = problem.AddResidualBlock(cost_function, loss_function, se3_vec.data());
        // }
        // else{
        //   residual_id = problem.AddResidualBlock(cost_function, nullptr, se3_vec.data());
        // }
        residual_id = problem.AddResidualBlock(cost_function, loss_function, se3_vec.data());
        residual_ids.emplace_back(residual_id);
      }
      else{
        residual_ids.emplace_back(nullptr);
      }

    }

    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";

    ceres::Problem::EvaluateOptions eval_options;
    eval_options.apply_loss_function = false;
    std::vector<double> costs, costs_for_threshold;

    for (size_t i = 0; i < residual_ids.size(); i++) {

      if(residual_ids[i]){

        const ceres::ResidualBlockId& residual_id = residual_ids[i];

        eval_options.residual_blocks = {residual_id};  // vector of ResidualBlockId

        std::vector<double> residual_errors;
        double cost = 0.0;

        problem.Evaluate(eval_options, &cost, &residual_errors, nullptr, nullptr);
        costs.emplace_back(cost);
        costs_for_threshold.emplace_back(cost);
      }      
      else{
        costs.emplace_back(-1);
      }

    }

    std::sort(costs_for_threshold.begin(), costs_for_threshold.end());
    size_t  max_cost_idx = static_cast<size_t>(0.98 * costs_for_threshold.size());
    size_t  threshold_idx = static_cast<size_t>(0.90 * costs_for_threshold.size());
    double max_cost = costs_for_threshold[max_cost_idx];
    double cost_threshold = costs_for_threshold[std::min(threshold_idx, costs_for_threshold.size() - 1)];
    if(max_cost < 4){
      break;
    }

    for(size_t i = 0; i < residual_ids.size(); i++){
      if(residual_ids[i]){
        double cost = costs[i];
        if(cost > cost_threshold){
          is_outlier_cost_functions[i] = true;
        }
      }
    }

  }

  int num_inlier = 0;
  int num_outlier = 0;

  for(int i=0;i<is_outlier_cost_functions.size();i++){
    if(is_outlier_cost_functions[i] == false){
      num_inlier++;
    } 
    else{
      num_outlier++;
    }
  }
  


  Tbw = Sophus::SE3d::exp(se3_vec);
  LOG(INFO) << GREEN << "only pose estimation: \n" << Tbw.matrix() << "\n inlier_num : " << num_inlier << "/" << num_outlier << RESET;


  return num_inlier;

}
    
} // namespace modules_vins





