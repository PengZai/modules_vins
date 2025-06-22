#pragma once


#include "tracker.h"


namespace modules_vins
{

class KLTTracker : public Tracker{


    public:
    KLTTracker(const std::shared_ptr<SystemConfig> &sys_config);


    void matching(const std::shared_ptr<Image> &img0, const std::shared_ptr<Image> &img1, 
        std::vector<cv::DMatch> &good_matches, 
        const int max_count, const float epsilon, const float y_distance_threshold = -1.0);


    void pipeline(const std::shared_ptr<CameraFrame> &camera_frame) override;
    void trackInFrame(const std::shared_ptr<CameraFrame> &camera_frame) override;

    protected:
    // int max_count_;	// Max number of iterations (used if COUNT is set)
    // double epsilon_; // Min required accuracy / threshold (used if EPS is set)
};


    
} // namespace modules_vins
