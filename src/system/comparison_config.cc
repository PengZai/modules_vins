#include "comparison_config.h"


namespace modules_vins{


ComparisonParameters::ComparisonParameters():
color_(3)
{

}

void ComparisonParameters::setName(const std::string name){
    this->name_ = name;
}

void ComparisonParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;

    parse("enable", this->enable_);
    parse("color", this->color_);
    parse("max_tolerant_time_offset", this->max_tolerant_time_offset_);
    
    parse("set_first_pose_in_origin", this->set_first_pose_in_origin_);
    parse("path", this->path_);
    parse("T_cam_comparison", this->T_cam_comparison_);



}












} //modules_vins