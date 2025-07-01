#pragma once


#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "../utils/utils.h"


namespace modules_vins
{

class ComparisonParameters : public Parameters
{

    public:
        ComparisonParameters();

        void loadFromNode(const std::shared_ptr<cv::FileNode> &node);

        void setName(const std::string name);
        
        bool enable_;
        std::string name_;
        double max_tolerant_time_offset_;
        Eigen::VectorXd color_;
        int set_first_pose_in_origin_;
        std::string path_;        
        Eigen::Matrix4d T_cam_comparison_;


    public:
       
        

};



class ComparisonConfig : public Config
{
    public:


    public:

        std::vector<std::shared_ptr<ComparisonParameters>> params_vector_;

 


}; 





} // namespace modules_vins