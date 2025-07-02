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

        void loadFromNode(const std::shared_ptr<cv::FileNode> &node) override;

        void setName(const std::string name);
        
        bool enable_;
        double max_tolerant_time_offset_;
        Eigen::VectorXd color_;
        int set_first_pose_in_origin_;
        std::string path_;        


    public:
       
        

};



class ComparisonConfig : public Config
{
    public:


    public:


 


}; 





} // namespace modules_vins