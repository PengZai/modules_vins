#include "camera.h"



namespace modules_vins{


// CameraData::CameraData(){};

// CameraData::CameraData(
//     const std::vector<float> &timestamp_vector, 
//     const std::vector<int> &sensor_id_vector, 
//     const std::vector<cv::Mat> &image_vector) : 
//     timestamp_vector_(timestamp_vector),
//     sensor_id_vector_(sensor_id_vector),
//     image_vector_(image_vector)
// {
//     if(timestamp_vector.size() > 0){
//         this->timestamp_ = timestamp_vector.at(0);
//     }

// };

int Image::id_counter_ = -1;

Image::Image(double timestamp, int sensor_id, cv::Mat data):
id_(++Image::id_counter_), 
timestamp_(timestamp), 
sensor_id_(sensor_id), 
color_data_(data)
{
    this->T_c_w_ = Sophus::SE3<double>();
    this->depth_.create(this->color_data_.rows, this->color_data_.cols, CV_64FC1);
    this->depth_.setTo(-1);

}


void Image::setKeyPoints(std::vector<cv::KeyPoint> &cv_key_points){

    this->keypoint_vector_.clear();
    appendKeyPoints(cv_key_points);
}

void Image::setKeyPoints(std::vector<cv::KeyPoint> &cv_key_points, const cv::Mat &descriptors){

    this->keypoint_vector_.clear();
    appendKeyPoints(cv_key_points, descriptors);
}


void Image::appendKeyPoints(std::vector<cv::KeyPoint> &cv_key_points){

    for(size_t i=0; i < cv_key_points.size(); i++){
        const std::shared_ptr<KeyPoint> &kp = std::make_shared<KeyPoint>(cv_key_points[i]);
        this->keypoint_vector_.emplace_back(kp);
    }

}


void Image::appendKeyPoints(std::vector<cv::KeyPoint> &cv_key_points, const cv::Mat &descriptors){

    for(size_t i=0; i < cv_key_points.size(); i++){
        const std::shared_ptr<KeyPoint> &kp = std::make_shared<KeyPoint>(cv_key_points[i]);
        kp->descriptor_ = descriptors.row(i);

        this->keypoint_vector_.emplace_back(kp);
    }

}

void Image::getCVKeyPoints(std::vector<cv::KeyPoint> &cv_key_points){
    for(size_t i=0; i < this->keypoint_vector_.size(); i++){
        if(this->keypoint_vector_.at(i) != nullptr){
            cv_key_points.emplace_back(this->keypoint_vector_.at(i)->cv_keypoint_);
        }
        else{
            cv_key_points.emplace_back(cv::KeyPoint());
        }
    }
}

void Image::getDescripots(cv::Mat &descriptors){
    
    std::vector<cv::Mat> descriptor_vec;
    for(size_t i=0; i < this->keypoint_vector_.size(); i++){
        const std::shared_ptr<KeyPoint> &kp = this->keypoint_vector_.at(i);

        if (!kp->descriptor_.empty()) {
            descriptor_vec.emplace_back(kp->descriptor_);
        }
        else{
            LOG(INFO) << RED << "there is a empty descriptor for keypoint " << i << "in image " << this->id_;
            return;
        }
    }

    cv::vconcat(descriptor_vec, descriptors);  

}


double Image::getPointDepthFromSensor(const cv::Point2f &pt){

    int x = cvRound(pt.x);
    int y = cvRound(pt.y);

    double depth_scale = 1.0;


    if(sensor_depth_.empty()){
        return 0;
    }

    // here, depth was stored as 32FC1, so we read data with float type.
    float d = sensor_depth_.at<float>(y, x);


    if ( d>0 && std::isnan(d) == false)
    {
        return double(d)/depth_scale;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = sensor_depth_.at<float>(y+dy[i], x+dx[i]);
            if ( d>0 && std::isnan(d) == false)
            {
                return double(d)/depth_scale;
            }
        }
    }
    return 0;
}


std::vector<Eigen::Vector3d> Image::getMapPoints() const {
    
    std::vector<Eigen::Vector3d> map_points;
    for(size_t i= 0;i< this->keypoint_vector_.size(); i++){
        const std::shared_ptr<KeyPoint> &kp = keypoint_vector_.at(i);
        if(kp->map_point_ptr_ != nullptr){
            map_points.push_back(kp->map_point_ptr_->pt3d_);
        }
    }

    return map_points;
}


void Image::setSensorDepth(const cv::Mat &sensor_depth){

    this->sensor_depth_ = sensor_depth.clone();
}


void Image::setTcw(const Sophus::SE3<double> T_c_w){

    this->T_c_w_ = T_c_w;


}

void Image::setTcw(const Eigen::Matrix3d &rotation, Eigen::Vector3d position){

    this->T_c_w_ = Sophus::SE3<double>(Sophus::SO3<double>(rotation), position);
}


void Image::cleanTrackInTimeRelationship(){
    this->matches_in_time_.clear();
    for(std::shared_ptr<KeyPoint> &keypoint : keypoint_vector_){
        
        keypoint->cleanTrackInTimeRelationship();
    }

}

void Image::cleanFeaturePoints(){

    this->keypoint_vector_.clear();

}


Eigen::Matrix3d Image::getRotation(){

    return T_c_w_.rotationMatrix();
}
Eigen::Vector3d Image::getPosition(){

    return T_c_w_.translation();
}


bool Image::isInImage(const cv::Point2d &pixel){

    return pixel.x > 0 && pixel.x < this->color_data_.cols && pixel.y >0 && pixel.y < this->color_data_.rows;

}


// just for auto incremental
int CameraFrame::id_counter_=-1;


CameraFrame::CameraFrame():
id_(++CameraFrame::id_counter_),
ref_camera_frame_(nullptr),
is_key_camera_frame_(false),
status_(CameraFrame::Status::NOT_INITIALIZED)
{


}

CameraFrame::CameraFrame(const std::vector<std::shared_ptr<Image>> image_vector):
CameraFrame()
{
    this->image_vector_ = image_vector;

}



void CameraFrame::cleanTrackInTimeRelationship(){

    for( std::shared_ptr<Image> image : this->image_vector_){
        image->cleanTrackInTimeRelationship();
    }
}

void CameraFrame::cleanFeaturePoints(){

    for( std::shared_ptr<Image> image : this->image_vector_){
        image->cleanFeaturePoints();
    }
}

void CameraFrame::setTrackInTimeRelationship(const std::vector<cv::DMatch> &matches){

    std::shared_ptr<Image> &img_0 = this->image_vector_.at(0);
    img_0->matches_in_time_.clear();
    for (size_t i=0; i < matches.size(); i++) {
        const cv::DMatch &match = matches[i];
        
        std::shared_ptr<KeyPoint> &tracked_keypoint_from_img_0 = img_0->keypoint_vector_[match.queryIdx];


        tracked_keypoint_from_img_0 ->setMatchInFrame(match);
        img_0->matches_in_time_.push_back(match);

    }
}


void CameraFrame::setTrackInFrameRelationship(const std::vector<cv::DMatch> &matches){

    std::shared_ptr<Image> &img_0 = this->image_vector_.at(0);
    img_0->matches_in_frame_.clear();
    for (size_t i=0; i < matches.size(); i++) {
            
            const cv::DMatch &match = matches[i];
            std::shared_ptr<KeyPoint> &tracked_keypoint_from_img_0 = img_0->keypoint_vector_[match.queryIdx];

            tracked_keypoint_from_img_0 ->setMatchInFrame(match);
            img_0->matches_in_frame_.push_back(match);
    }
}



// Overload operator<< for logging
std::ostream& operator<<(std::ostream& os, const Image &img) {

    os << "Image Info: \n" 
    << "Timestamp: " << img.timestamp_ << "\n" 
    << "Sensor ID: " << img.sensor_id_ << "\n"
    << "Data Size: " << img.color_data_.rows << "x" << img.color_data_.cols << "\n"
    << "Keypoints Count: " << img.keypoint_vector_.size() << "\n";
    return os;
}



// // just for auto incremental
// int KeyCameraFrame::id_counter_=-1;


// KeyCameraFrame::KeyCameraFrame():
// CameraFrame()
// {
//     id_ = ++KeyCameraFrame::id_counter_;

// }

// KeyCameraFrame::KeyCameraFrame(const std::vector<std::shared_ptr<Image>> image_vector):
// KeyCameraFrame()
// {
//     this->image_vector_ = image_vector;

// }


} //modules_vins
 