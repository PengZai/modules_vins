#pragma once


#include "tracker.h"


namespace modules_vins
{

class KLTTracker : public Tracker{


    public:
    KLTTracker(const std::shared_ptr<SystemConfig> &sys_config);

    void matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, 
        std::vector<cv::DMatch> &good_matches, 
        const float error_threshold = -1.0, const float y_distance_threshold = -1.0) override;

};


    
} // namespace modules_vins
