#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "../../data/camera.h"
#include "../../system/system_config.h"


namespace modules_vins{



class FeaturePoint{

    public:
    FeaturePoint(const std::shared_ptr<SystemConfig> &sys_config);
    virtual ~FeaturePoint() = default;


    virtual void detect(const std::shared_ptr<Image> &img) = 0;
    virtual void pipeline(const std::shared_ptr<CameraFrame> &camera_frame) = 0;

    protected:
    int num_features_;
};


class ORBFeature : public FeaturePoint{

    public:
    ORBFeature(const std::shared_ptr<SystemConfig> &sys_config);

    void detect(const std::shared_ptr<Image> &img) override;
    void pipeline(const std::shared_ptr<CameraFrame> &camera_frame) override;

    protected:
    int num_features_;
    double scale_factor_;
    int level_pyramid_;
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::GFTTDetector> gftt_;


};

class GoodFeature : public FeaturePoint{


    public:
    GoodFeature(const std::shared_ptr<SystemConfig> &sys_config);
    void detect(const std::shared_ptr<Image> &img) override;
    void pipeline(const std::shared_ptr<CameraFrame> &camera_frame) override;


    protected:
    cv::Ptr<cv::GFTTDetector> gftt_;


};







} //modules_vins