#include "camera.h"


namespace modules_vins{


// CameraData::CameraData(){};

// CameraData::CameraData(
//     const std::vector<double> &timestamp_vector, 
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
id_(++Image::id_counter_), timestamp_(timestamp), sensor_id_(sensor_id), data_(data)
{

}


std::vector<cv::Point3f> Image::getMapPoints() const {
    
    std::vector<cv::Point3f> map_points;
    for(int i = 0;i<this->keypoint_vector_.size(); i++){
        const std::shared_ptr<KeyPoint> &kp = keypoint_vector_.at(i);
        if(kp->map_point_ptr_ != nullptr){
            map_points.push_back(kp->map_point_ptr_->pt_);
        }
    }

    return map_points;
}


// just for auto incremental
int CameraFrame::id_counter_=-1;


CameraFrame::CameraFrame():
id_(++CameraFrame::id_counter_)
{


}

CameraFrame::CameraFrame(const std::vector<std::shared_ptr<Image>> image_vector):
image_vector_(image_vector), id_(++CameraFrame::id_counter_)
{

}


void CameraFrame::setMap(const std::shared_ptr<Map> &map){

    this->map_ = map;
}

// CameraFrame::CameraFrame(const CameraFrame &camera_frame):
// image_vector_(camera_frame.image_vector_), id_(camera_frame.id_)
// {
//     CameraFrame::id_counter_++;
//     this->id_ = id_counter_;
// }



// Overload operator<< for logging
std::ostream& operator<<(std::ostream& os, const Image &img) {
    os << "Image Info: \n" 
    << "Timestamp: " << img.timestamp_ << "\n" 
    << "Sensor ID: " << img.sensor_id_ << "\n"
    << "Data Size: " << img.data_.rows << "x" << img.data_.cols << "\n"
    << "Keypoints Count: " << img.keypoint_vector_.size() << "\n"
    << "Descriptor Size: " << img.descriptors_.rows << "x" << img.descriptors_.cols << "\n";
    return os;
}


} //modules_vins
 