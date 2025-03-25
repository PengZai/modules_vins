#include "state.h"

namespace modules_vins{

State::State(std::shared_ptr<SystemConfig> sys_config)
:sys_config_(sys_config)
{
    this->rotation_ = Eigen::Matrix<double, 3, 3>::Identity();
    this->position_ = Eigen::Vector3d::Zero();


}



} //namespace modules_vins