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
    void publish_cross_frame(CameraFrame camera_frame);
    void publish_cross_time(CameraFrame camera_frame);


    public:
    std::shared_ptr<SystemConfig> sys_config_;
    
    protected:
    std::deque<CameraFrame> camera_frame_deque_;
    std::deque<cv::Mat> img_deque_;


    cv::Scalar GreenColor_; // green
    cv::Scalar RedColor_; // red
    cv::Scalar BlueColor_; // blue

};
    
} // namespace modules_vins

