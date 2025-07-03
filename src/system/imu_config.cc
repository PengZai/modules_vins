#include "imu_config.h"


namespace modules_vins
{
    


ImuParameters::ImuParameters()
{

}



void ImuParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;


    parse("name", this->name_);
    parse("T_imu0_imu", this->T_imu0_sensor_);
}


} // namespace modules_vins