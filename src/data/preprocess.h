#pragma once

#include "camera.h"


namespace modules_vins{

class DataPreprocesor{

    public:

    void pipeline(const std::shared_ptr<CameraFrame> &camera_frame);


};



} //modules_vins