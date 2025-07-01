#include "feature_points.h"

namespace modules_vins{


FeaturePoint::FeaturePoint(const std::shared_ptr<SystemConfig> &sys_config){

    this->sys_config_ = sys_config;


}



ORBFeature::ORBFeature(const std::shared_ptr<SystemConfig> &sys_config):
FeaturePoint(sys_config)
{

    this->num_feature_points_ = sys_config->feature_and_tracker_config_->orb_params_->num_feature_points_;
    this->block_size_ = sys_config->feature_and_tracker_config_->orb_params_->block_size_;
    this->fastThreshold_ = sys_config->feature_and_tracker_config_->orb_params_->fastThreshold_;
    this->useNonmaxSuppression_ = sys_config->feature_and_tracker_config_->orb_params_->useNonmaxSuppression_;;

    this->orb_ = cv::ORB::create();
}

// void ORBFeature::detect(const std::shared_ptr<Image> &img){
  

//     // cv::Mat mask(img->gray_data_.size(), CV_8UC1, 255);

//     // for (std::shared_ptr<KeyPoint> &kp : img->keypoint_vector_) {
//     //     cv::rectangle(mask, kp->pt2i_ - cv::Point2f(10, 10),
//     //                   kp->pt2i_ + cv::Point2f(10, 10), 0, cv::FILLED);
//     // }

//     std::vector<cv::KeyPoint> cv_key_points;

//     this->gftt_->detect(img->gray_data_, cv_key_points);
//     // this->orb_->detect(img->gray_data_, cv_key_points);

    
//     cv::Mat descriptors;
//     this->orb_->compute(img->gray_data_, cv_key_points, descriptors);

//     cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVIntrinsicsMatrix();         
//     cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVDistortionCoeffs();


//     img->appendAndUndistotKeyPoints(cv_key_points, descriptors, cv_K, cv_distortion_coeffs);
//     LOG(INFO) << " we leave " << img->keypoint_vector_.size() << " features after descriptor computation";

    

// }

void ORBFeature::detect(const std::shared_ptr<Image> &img){
  

    std::vector<cv::KeyPoint> cv_key_points;
    int blockSize = this->block_size_;
    int rows = img->gray_data_.rows/blockSize;
    int cols = img->gray_data_.cols/blockSize;
    const int max_num_per_cell = std::max(1, this->num_feature_points_/(rows*cols));


    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int x = j * blockSize;
            int y = i * blockSize;
            int w = std::min(blockSize, img->gray_data_.cols - x);
            int h = std::min(blockSize, img->gray_data_.rows - y);
            cv::Rect BlockRect(x, y, w, h);
            
            cv::Mat blockROI = img->gray_data_(BlockRect);
            
            std::vector<cv::KeyPoint> blockKps;
            // this->orb_->detect(blockROI, blockKps);
            cv::FAST(blockROI, blockKps, this->fastThreshold_, this->useNonmaxSuppression_);

            
            std::sort(blockKps.begin(), blockKps.end(),
                [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                    return a.response > b.response;
                });

            // Keep at most N strongest and offset to full image coordinates
            int count = std::min(max_num_per_cell, (int)blockKps.size());
            for (int k = 0; k < count; ++k) {
                blockKps[k].pt.x += x;
                blockKps[k].pt.y += y;
                cv_key_points.push_back(blockKps[k]);
            }
        }
    }



    
    cv::Mat descriptors;
    this->orb_->compute(img->gray_data_, cv_key_points, descriptors);

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVIntrinsicsMatrix();         
    cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVDistortionCoeffs();


    img->appendAndUndistotKeyPoints(cv_key_points, descriptors, cv_K, cv_distortion_coeffs);
    LOG(INFO) << " we leave " << img->keypoint_vector_.size() << " features after descriptor computation";

    

}



void ORBFeature::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){
    
    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        return;
    }


    for(size_t i=0;i<camera_frame->image_vector_.size();i++){

        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);
        detect(img_i);

        LOG(INFO) << GREEN << "Detect " << img_i->keypoint_vector_.size() << " new features" << RESET;


    }


}




    

GoodFeature::GoodFeature(const std::shared_ptr<SystemConfig> &sys_config):
FeaturePoint(sys_config)
{

    this->num_feature_points_ = this->sys_config_->feature_and_tracker_config_->klt_params_->num_feature_points_;
    this->min_distance_ = this->sys_config_->feature_and_tracker_config_->klt_params_->min_distance_;
    this->gftt_ = cv::GFTTDetector::create(this->num_feature_points_, 0.01, this->min_distance_*2);


}


void GoodFeature::detect(const std::shared_ptr<Image> &img){


    std::vector<cv::KeyPoint> cv_key_points;
    this->gftt_ = cv::GFTTDetector::create(this->num_feature_points_ - img->keypoint_vector_.size(), 0.01, this->min_distance_*2);


    cv::Mat mask(img->gray_data_.size(), CV_8UC1, 255);
    for (auto &kp : img->keypoint_vector_) {
        cv::rectangle(mask, kp->cv_keypoint_.pt - cv::Point2f(this->min_distance_, this->min_distance_),
                      kp->cv_keypoint_.pt + cv::Point2f(this->min_distance_, this->min_distance_), 0, cv::FILLED);
    }
    // cv::imshow("mask", mask);
    // cv::waitKey(0);

    this->gftt_->detect(img->gray_data_, cv_key_points, mask);

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVIntrinsicsMatrix();         
    cv::Mat cv_distortion_coeffs = this->sys_config_->camera_config_->params_vector_.at(img->sensor_id_)->getCVDistortionCoeffs();

    img->appendAndUndistotKeyPoints(cv_key_points, cv_K, cv_distortion_coeffs);


}


void GoodFeature::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){
    
    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        return;
    }



    const std::shared_ptr<Image> &img_0 = camera_frame->image_vector_.at(0);

    // if(img_0->keypoint_vector_.size() < this->num_feature_points_ ){
    //     detect(img_0);
    // }
    
    detect(img_0);


    LOG(INFO) << GREEN << "Detected " << img_0->keypoint_vector_.size() << " features" << RESET;

    


}


    

   

} //modules_vins