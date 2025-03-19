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

Image::Image(double timestamp, int sensor_id, cv::Mat data):
timestamp_(timestamp), sensor_id_(sensor_id), data_(data)
{

}



CameraFrame::CameraFrame(){
    
}

CameraFrame::CameraFrame(const std::vector<Image> image_vector):
image_vector_(image_vector)
{

}


// Overload operator<< for logging
std::ostream& operator<<(std::ostream& os, const Image& img) {
    os << "Image Info: \n" 
    << "Timestamp: " << img.timestamp_ << "\n" 
    << "Sensor ID: " << img.sensor_id_ << "\n"
    << "Data Size: " << img.data_.rows << "x" << img.data_.cols << "\n"
    << "Keypoints Count: " << img.keypoint_vector_.size() << "\n"
    << "Descriptor Size: " << img.descriptors_.rows << "x" << img.descriptors_.cols << "\n";
    return os;
}


} //modules_vins
 