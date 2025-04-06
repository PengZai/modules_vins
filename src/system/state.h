#pragma once


#include<memory>


#include "../system/system_config.h"
#include "../data/camera.h"

namespace modules_vins{

class State{

    public:
        std::vector<Sophus::SE3<double>> T_c_w_vector_; // the vector of pose of robot in world coordinate
        std::shared_ptr<Map> map_;

};


} //namespace modules_vins