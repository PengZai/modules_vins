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

};
    
} // namespace modules_vins

