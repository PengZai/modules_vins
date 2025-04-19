#pragma once

#include <memory>

#ifdef USE_LIBTORCH
#include "depth_estimations/MiDas_reconstruction.h"
#endif

#include "two_view_depths/two_view_reconstruction.h"
#include "sensor_depths/sensor_depth_reconstruction.h"




#include "../data/camera.h"
#include "../system/system_config.h"
#include "../data/camera.h"
#include "../data/map.h"




namespace modules_vins
{


class Reconstructor{

    public:

    Reconstructor(const std::shared_ptr<SystemConfig> &sys_config);
    void pipeline(std::shared_ptr<CameraFrame> &camera_frame);




    protected:
    std::shared_ptr<SystemConfig> sys_config_;


    std::shared_ptr<TwoViewReconstructor> two_view_reconstructor_;
    std::shared_ptr<SensorDepthReconstruction> sensor_depth_reconstructor_;

    #ifdef USE_LIBTORCH 
    std::shared_ptr<MiDas> midas_reconstructor_;
    #endif


};
    
} // namespace modules_vins


