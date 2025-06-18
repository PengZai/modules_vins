#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <memory>

#ifdef USE_LIBTORCH
#include "object_detections/yolo_detector.h"
#include "semantic_segmentations/yolo_segmentor.h"
#endif

#include "../data/camera.h"
#include "../system/system_config.h"
#include "features/feature_points.h"





namespace modules_vins
{

class Detector{

    public:
    Detector(const std::shared_ptr<SystemConfig> &sys_config);
    void setFeaturePoint(const std::shared_ptr<FeaturePoint> &feature_point);
    void pipeline(std::shared_ptr<CameraFrame> &camera_frame);
    void detect(const std::shared_ptr<Image> &img);
    void computeDescriptor(const std::shared_ptr<Image> &img);


    protected:
    std::shared_ptr<FeaturePoint> feature_point_;

    #ifdef USE_LIBTORCH
    std::shared_ptr<YOLODetector> yolo_detector_;
    std::shared_ptr<YOLOSegmentor> yolo_segmentor_;
    #endif
    
    std::shared_ptr<SystemConfig> sys_config_;

};

    
} // namespace modules_vins

