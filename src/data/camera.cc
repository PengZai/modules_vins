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

int Image::image_id_counter_ = -1;

Image::Image(double timestamp, int sensor_id, cv::Mat data):
timestamp_(timestamp), sensor_id_(sensor_id), data_(data), image_id_(++Image::image_id_counter_)
{

}

// just for auto incremental
int CameraFrame::frame_id_counter_=-1;


CameraFrame::CameraFrame():
frame_id_(++CameraFrame::frame_id_counter_)
{


}

CameraFrame::CameraFrame(const std::vector<Image> image_vector):
image_vector_(image_vector), frame_id_(++CameraFrame::frame_id_counter_)
{

}

// CameraFrame::CameraFrame(const CameraFrame &camera_frame):
// image_vector_(camera_frame.image_vector_), frame_id_(camera_frame.frame_id_)
// {
//     CameraFrame::frame_id_counter_++;
//     this->frame_id_ = frame_id_counter_;
// }



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
 