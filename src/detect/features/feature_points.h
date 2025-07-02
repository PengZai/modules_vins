#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "../../data/frame.h"
#include "../../system/system_config.h"


namespace modules_vins{



class FeaturePoint{

    public:
    FeaturePoint(const std::shared_ptr<SystemConfig> &sys_config);
    virtual ~FeaturePoint() = default;


    virtual void detect(const std::shared_ptr<Image> &img) = 0;
    virtual void pipeline(const std::shared_ptr<Frame> &frame) = 0;

    protected:
    std::shared_ptr<SystemConfig> sys_config_;
    int num_feature_points_;

};


class ORBFeature : public FeaturePoint{

    public:
    ORBFeature(const std::shared_ptr<SystemConfig> &sys_config);

    void detect(const std::shared_ptr<Image> &img) override;
    void pipeline(const std::shared_ptr<Frame> &frame) override;

    protected:
    int block_size_;
    int fastThreshold_;
    bool useNonmaxSuppression_;
    // double scale_factor_;
    // int level_pyramid_;
    cv::Ptr<cv::ORB> orb_;


};

class GoodFeature : public FeaturePoint{


    public:
    GoodFeature(const std::shared_ptr<SystemConfig> &sys_config);
    void detect(const std::shared_ptr<Image> &img) override;
    void pipeline(const std::shared_ptr<Frame> &frame) override;


    protected:
    cv::Ptr<cv::GFTTDetector> gftt_;
    int min_distance_;



};







} //modules_vins