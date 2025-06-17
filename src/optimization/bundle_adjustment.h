#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h> 
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace modules_vins
{
    

void bundleAdjustment(Sophus::SE3<double> &Transformation, std::vector<Eigen::Vector3d> &pts3d, std::vector<Eigen::Vector2d> &observation_pts2d, const Eigen::Matrix3d &K, const Eigen::VectorXd &distortion_coeffs);

struct ReprojectionError {
    ReprojectionError(double obs_x, double obs_y,
                      const Eigen::Matrix3d& K,
                      const Eigen::VectorXd& dist)
        : observed_x(obs_x), observed_y(obs_y), fx(K(0, 0)), fy(K(1, 1)),
          cx(K(0, 2)), cy(K(1, 2)),
          k1(dist(0)), k2(dist(1)) {}
  
    template <typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const {
      // Rotate and translate the point
      T p[3];
      ceres::AngleAxisRotatePoint(camera, point, p);
      p[0] += camera[3];
      p[1] += camera[4];
      p[2] += camera[5];
  
      // Normalize to get (x, y)
      T x = p[0] / p[2];
      T y = p[1] / p[2];
  
      // Apply radial distortion
      T r2 = x * x + y * y;
      T distortion = T(1.0) + k1 * r2 + k2 * r2 * r2;
  
      T x_distorted = distortion * x;
      T y_distorted = distortion * y;
  
      // Project to image plane using intrinsics
      T u = fx * x_distorted + cx;
      T v = fy * y_distorted + cy;
  
      // Residuals
      residuals[0] = u - T(observed_x);
      residuals[1] = v - T(observed_y);
      return true;
    }
  
    static ceres::CostFunction* Create(const double observed_x, const double observed_y,
                                       const Eigen::Matrix3d& K,
                                       const Eigen::VectorXd& dist) {
      return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
          new ReprojectionError(observed_x, observed_y, K, dist)));
    }
  
    double observed_x, observed_y;
    double fx, fy, cx, cy;
    double k1, k2;
  };


} // namespace modules_vins
