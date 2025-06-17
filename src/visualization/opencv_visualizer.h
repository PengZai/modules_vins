

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "../log/logging.h"
#include "../system/system_config.h"
#include "../data/map.h"
#include "../data/camera.h"

namespace modules_vins
{

class OpenCVVisualizer{


    public:

    OpenCVVisualizer(const std::shared_ptr<SystemConfig> &config);
    void publish(const std::shared_ptr<CameraFrame> &camera_frame);
    void setMap(const std::shared_ptr<Map> &map);
    void drawTrackingPointPattern(cv::Mat &img, const std::shared_ptr<KeyPoint> &keypoint, const cv::Scalar &color);
    void publishMatchingInFrame(const std::shared_ptr<CameraFrame> &camera_frame);
    void publishMatchingInTime(const std::shared_ptr<CameraFrame> &camera_frame);
    void publishProjectedMapPoint(const std::shared_ptr<CameraFrame> &camera_frame);
    void publishStereoDepth(const std::shared_ptr<CameraFrame> &camera_frame);
    void publishLearnedStereoDisparity(const std::shared_ptr<CameraFrame> &camera_frame);
    void publishSensorDepth(const std::shared_ptr<CameraFrame> &camera_frame);
    void publishLearnedDepth(const std::shared_ptr<CameraFrame> &camera_frame);
    void invDepthAndMixColor(const cv::Mat &input_depth, const cv::Mat &input_color, cv::Mat &mixed_depth_color);
    void publishObjectDetection(const std::shared_ptr<CameraFrame> &camera_frame);
    void publishSemanticSegmentation(const std::shared_ptr<CameraFrame> &camera_frame);
    void publishTrackingInTime(const std::shared_ptr<CameraFrame> &camera_frame);


    public:
    std::shared_ptr<SystemConfig> sys_config_;
    
    protected:


    cv::Scalar GreenColor_; // green
    cv::Scalar RedColor_; // red
    cv::Scalar BlueColor_; // blue
    std::shared_ptr<Map> map_;

    std::vector<std::string> classes_;
    std::vector<cv::Scalar> class_colors_;


};
    
} // namespace modules_vins

