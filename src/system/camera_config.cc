#include "camera_config.h"


namespace modules_vins
{





 




CameraParameters::CameraParameters():
    resolution_(2), 
    distortion_coeffs_(4), 
    intrinsics_(4),
    use_sensor_depth_(false),
    use_learned_depth_(false),
    use_stereo_matching_(false),
    use_learned_stereo_matching_(false),
    use_learned_object_detection_(false),
    use_learned_semantic_segmentation_(false)
{

}



// void CameraParameters::createMapFrompixel2UndistoredNormalizedPlane(){

//     const int w = resolution_(0);
//     const int h = resolution_(1);
//     // const double fx = intrinsics_(0);
//     // const double fy = intrinsics_(1);
//     // const double cx = intrinsics_(2);
//     // const double cy = intrinsics_(3);

//     // const double k1 = distortion_coeffs_(0);
//     // const double k2 = distortion_coeffs_(1);
//     // const double p1 = distortion_coeffs_(2);
//     // const double p2 = distortion_coeffs_(3);

//     cv::Mat cv_K = getCVIntrinsicsMatrix();
//     cv::Mat cv_distortion_coeffs = getCVDistortionCoeffs();
//     std::vector<cv::Point2f> pixels;
//     std::vector<cv::Point2f> undistorted_points;

//     MapVU2UndisXY_ = std::make_shared<Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, Eigen::Dynamic>>(h, w);

//     for(int v=0;v<h;v++){
//         for(int u=0;u<w;u++){
//             pixels.push_back(cv::Point2f(u, v));
//         }
//     }
//     cv::undistortPoints(pixels, undistorted_points, cv_K, cv_distortion_coeffs);

//     int idx = 0;
//     for(int v=0;v<h;v++){
//         for(int u=0;u<w;u++){
//             (*MapVU2UndisXY_)(v,u) = Eigen::Vector2d(undistorted_points[idx].x, undistorted_points[idx].y);
//             idx++;
//         }
//     }


// }


const Eigen::VectorXd CameraParameters::getDistortionCoeffs(){

    return this->distortion_coeffs_;
}

const cv::Mat CameraParameters::getCVDistortionCoeffs() {

    cv::Mat cv_distortion_coeffs;

    cv::eigen2cv(this->distortion_coeffs_, cv_distortion_coeffs);

    return cv_distortion_coeffs;
}


const Eigen::Matrix3d CameraParameters::getIntrinsicsMatrix(){


    Eigen::Matrix3d K;
    K << this->intrinsics_(0), 0, this->intrinsics_(2),
         0, this->intrinsics_(1), this->intrinsics_(3),
         0, 0, 1;

    return K;
}


const cv::Mat CameraParameters::getCVIntrinsicsMatrix(){
    
    const Eigen::Matrix3d K = getIntrinsicsMatrix();

    cv::Mat cv_K;
    cv::eigen2cv(K, cv_K);

    return cv_K;

}



void CameraParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node)
{

    this->node_ = node;

    parse("rgb_rostopic", this->rgb_rostopic_);

    parse("use_sensor_detph", this->use_sensor_depth_);
    parse("sensor_depth_rostopic", this->sensor_depth_rostopic_);
    
    parse("use_learned_depth", this->use_learned_depth_);
    parse("model_name_learned_depth", this->model_name_learned_depth_);
    parse("output_learned_depth_rostopic", this->output_learned_depth_rostopic_);

    parse("use_stereo_matching", this->use_stereo_matching_);
    parse("stereo_matching_rostopic", this->stereo_matching_rostopic_);

    parse("use_learned_stereo_matching", this->use_learned_stereo_matching_);
    parse("model_name_learned_stereo_matching", this->model_name_learned_stereo_matching_);
    parse("learned_stereo_matching_rostopic", this->learned_stereo_matching_rostopic_);

    parse("use_learned_object_detection", this->use_learned_object_detection_);
    parse("model_name_learned_object_detection", this->model_name_learned_object_detection_);
    parse("learned_object_detection_rostopic", this->learned_object_detection_rostopic_);

    parse("use_learned_semantic_segmentation", this->use_learned_semantic_segmentation_);
    parse("model_name_learned_semantic_segmentation", this->model_name_learned_semantic_segmentation_);
    parse("learned_semantic_segmentation_rostopic", this->learned_semantic_segmentation_rostopic_);


  

    parse("output_rostopic", this->output_rostopic_);
    parse("camera_model", this->camera_model_);
    parse("distortion_model_", this->distortion_model_);

    parse("resolution", this->resolution_);
    parse("distortion_coeffs", this->distortion_coeffs_);
    parse("intrinsics", this->intrinsics_);

    parse("T_imu_cam", this->T_imu_cam_);



}



} // namespace modules_vins