#include "frame.h"



namespace modules_vins {




// just for auto incremental
int Frame::id_counter_=-1;


Frame::Frame():
id_(++Frame::id_counter_),
ref_frame_(nullptr),
is_key_frame_(false),
status_(Frame::Status::NOT_INITIALIZED)
{

    this->T_b_w_ = Sophus::SE3<double>();
    this->Velocity_T_b_w_.setZero();

}

// Frame::Frame(const std::vector<std::shared_ptr<Image>> image_vector):
// Frame()
// {
//     this->image_vector_ = image_vector;

// }

void Frame::setImages(const std::vector<std::shared_ptr<Image>> &image_vector){

    this->image_vector_ = image_vector;

}

// initialized Tcw with veclocity according to reference frame;
void Frame::initializeTbwWithVelocity(){

    if(this->ref_frame_ != nullptr){
        const double dt = this->timestamp_ - this->ref_frame_->timestamp_;
        // const double dt = 1;
        Sophus::SE3d relative_T_curr_ref = Sophus::SE3d::exp( this->ref_frame_->Velocity_T_b_w_ * dt);
        // this->setTsensorwWithTbw(relative_T_curr_ref * this->ref_frame_->T_b_w_);
        this->setTbw(relative_T_curr_ref * this->ref_frame_->T_b_w_);
        LOG(INFO) << GREEN << "initialize Tcw with velocity with :\n" << this->T_b_w_.matrix() << RESET;
    }

}

void Frame::cleanTrackInFrameRelationship(){

    for( std::shared_ptr<Image> image : this->image_vector_){
        image->cleanTrackInFrameRelationship();
    }
}

void Frame::cleanTrackInTimeRelationship(){

    for( std::shared_ptr<Image> image : this->image_vector_){
        image->cleanTrackInTimeRelationship();
    }
}

void Frame::cleanFeaturePoints(){

    for( std::shared_ptr<Image> image : this->image_vector_){
        image->cleanFeaturePoints();
    }
}

void Frame::setTrackInTimeRelationship(const std::vector<cv::DMatch> &matches){

    std::shared_ptr<Image> &img_0 = this->image_vector_.at(0);
    img_0->matches_in_time_.clear();
    for (size_t i=0; i < matches.size(); i++) {
        const cv::DMatch &match = matches[i];
        
        std::shared_ptr<KeyPoint> &tracked_keypoint_from_img_0 = img_0->keypoint_vector_[match.queryIdx];


        tracked_keypoint_from_img_0 ->setMatchInTime(match);
        img_0->matches_in_time_.push_back(match);

    }
}


void Frame::setTrackInFrameRelationship(const std::vector<cv::DMatch> &matches){

    std::shared_ptr<Image> &img_0 = this->image_vector_.at(0);
    img_0->matches_in_frame_.clear();
    for (size_t i=0; i < matches.size(); i++) {
            
            const cv::DMatch &match = matches[i];
            std::shared_ptr<KeyPoint> &tracked_keypoint_from_img_0 = img_0->keypoint_vector_[match.queryIdx];

            tracked_keypoint_from_img_0 ->setMatchInFrame(match);
            img_0->matches_in_frame_.push_back(match);
    }
}

// void Frame::setTsensorwWithTbw(const Sophus::SE3d &Tbw){

//     std::shared_ptr<Image> &img_0 = image_vector_.at(0);
//     img_0->setTcw(Tc0w);
    
//     for(size_t i=1; i<this->image_vector_.size();i++){

//         std::shared_ptr<Image> &img_i = image_vector_.at(i);
//         img_i->setTcw(img_i->T_c_c0_ * img_0->T_c_w_);
//     }
// }


void Frame::setTbw(const Sophus::SE3<double> T_b_w){
    this->T_b_w_ = T_b_w;
}

void Frame::setTbw(const Eigen::Matrix3d &rotation, Eigen::Vector3d position){
    this->T_b_w_ = Sophus::SE3<double>(Sophus::SO3<double>(rotation), position);
}


void Frame::setVelocity(const Sophus::Vector6d Velocity_T_b_w){
    this->Velocity_T_b_w_ = Velocity_T_b_w;
}

void Frame::calculateVelocityWithRefFrame(){

    if(this->ref_frame_){

        const Sophus::SE3d T_curr_ref =  this->T_b_w_ * ref_frame_->T_b_w_.inverse();

        double norm_T_curr_ref = T_curr_ref.log().norm();

        const double dt = this->timestamp_ - ref_frame_->timestamp_;

        const Sophus::Vector6d Velocity_T_b_w = T_curr_ref.log()/dt;
        this->setVelocity(Velocity_T_b_w);

    }
    else{
        LOG(INFO) << YELLOW << "lack of reference camera frame to set camera 0 velocity " << RESET;
    }
    
}
    

void Frame::propogateMappointWitchMatchInTimeRelationship(){

    if(this->ref_frame_ != nullptr){
        const std::shared_ptr<Frame> &ref_frame = this->ref_frame_;
        for(int i=0; i<ref_frame->image_vector_.at(0)->matches_in_time_.size(); i++){

            const std::shared_ptr<Image> img_0_from_ref_frame = ref_frame->image_vector_.at(0);
            const std::shared_ptr<Image> img_0_from_frame = this->image_vector_.at(0);
            const cv::DMatch &match = img_0_from_ref_frame->matches_in_time_.at(i);
            const std::shared_ptr<KeyPoint> kp_from_img_0_ref_frame = img_0_from_ref_frame->keypoint_vector_.at(match.queryIdx);

            if(kp_from_img_0_ref_frame->map_point_ptr_ != nullptr){
                const std::shared_ptr<KeyPoint> kp_from_img_0_frame = img_0_from_frame->keypoint_vector_.at(match.trainIdx);
                kp_from_img_0_frame->map_point_ptr_ = kp_from_img_0_ref_frame->map_point_ptr_;
            }

            if(kp_from_img_0_ref_frame->map_point_ptr2_ != nullptr){
                const std::shared_ptr<KeyPoint> kp_from_img_0_frame = img_0_from_frame->keypoint_vector_.at(match.trainIdx);
                kp_from_img_0_frame->map_point_ptr2_ = kp_from_img_0_ref_frame->map_point_ptr2_;
            }
        }
    }
    
}


}

