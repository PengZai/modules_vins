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

    public:
    std::shared_ptr<SystemConfig> config_;
    
    protected:
    std::deque<CameraFrame> camera_frame_deque_;

    cv::Scalar GreenColor_; // green
    cv::Scalar RedColor_; // red
    cv::Scalar BlueColor_; // blue

};
    
} // namespace modules_vins

