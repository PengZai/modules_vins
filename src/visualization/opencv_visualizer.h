

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "../log/logging.h"
#include "../system/system_config.h"
#include "../data/camera.h"

namespace modules_vins
{

class OpenCVVisualizer{


    public:

    OpenCVVisualizer(const std::shared_ptr<SystemConfig> &config);
    void publish(const CameraFrame &camera_frame);
    void drawTrackingPointPattern(cv::Mat &img, const std::shared_ptr<KeyPoint> &keypoint, const cv::Scalar &color);
    void publishMatchingInFrame(CameraFrame camera_frame);
    void publishMatchingInTime(CameraFrame camera_frame);
    void publishProjectedMapPoint(const CameraFrame &camera_frame);
    void publishSensorDepth(const CameraFrame &camera_frame);
    void publishLearnedDepth(const CameraFrame &camera_frame);
    void publishObjectDetection(const CameraFrame &camera_frame);
    void publishSemanticSegmentation(const CameraFrame &camera_frame);
    void publishTrackingInTime(const CameraFrame &camera_frame);


    public:
    std::shared_ptr<SystemConfig> sys_config_;
    
    protected:
    std::deque<CameraFrame> camera_frame_deque_;


    cv::Scalar GreenColor_; // green
    cv::Scalar RedColor_; // red
    cv::Scalar BlueColor_; // blue
 
    std::vector<std::string> classes_;

};
    
} // namespace modules_vins

