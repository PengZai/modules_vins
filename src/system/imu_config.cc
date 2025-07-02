#include "imu_config.h"


namespace modules_vins
{
    


ImuParameters::ImuParameters()
{

}



void ImuParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;

}


} // namespace modules_vins