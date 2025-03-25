#pragma once


#include<memory>


#include "../system/system_config.h"
#include "../data/camera.h"

namespace modules_vins{

class State{

    public:
    State(std::shared_ptr<SystemConfig> sys_config);
    
    std::shared_ptr<SystemConfig> sys_config_;



    protected:
    Eigen::Matrix<double, 3, 3> rotation_;
    Eigen::Vector3d position_;



};


} //namespace modules_vins