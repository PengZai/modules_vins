#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "../../../data/camera.h"


namespace modules_vins{


class ORBFeature{

    public:
    ORBFeature(int num_features);

    void detect(Image &img);
    void compute(Image &img);

    protected:
    int num_features_;
    cv::Ptr<cv::ORB> orb;


};

class FASTFeature{


    public:
    FASTFeature(int num_features);

    void detect(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints);

    protected:
    cv::Ptr<cv::FastFeatureDetector> fast;
    int num_features_;
 
};

// ShiTomasiCorner
class STCornerFeature{


    

    public:
    STCornerFeature(int num_features);

    void detect(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints);


    protected:
    int num_features_;
    

};


} //modules_vins