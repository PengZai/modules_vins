#include "map.h"


namespace modules_vins
{


Map::Map(const std::shared_ptr<SystemConfig> &sys_config)
:sys_config_(sys_config)
{

}

const std::map<unsigned int, std::shared_ptr<MapPoint>>& Map::getMapPoints() const{
    return this->mappoints_;
}

bool Map::isExistedMapPoint(const std::shared_ptr<MapPoint> &mappoint){

    auto it = this->mappoints_.find(mappoint->id_);
    if (it != this->mappoints_.end()) {
        return true;
    } else {
        return false;
    }

}

void Map::insertMapPoint(const std::shared_ptr<MapPoint> &mappoint){

    this->mappoints_.insert({mappoint->id_, mappoint});


}





void Map::update(const std::shared_ptr<Frame> &frame){

    if(frame->status_ != Frame::Status::NORMAL){
        return;
    }

    if(frame->image_vector_.size()==0){

        LOG(INFO) << RED << "size of image vector equal to 0" << RESET;
        std::exit(EXIT_FAILURE);
    }

    const std::shared_ptr<Image> &img_0 = frame->image_vector_.at(0);

    // project 3d point in camera coordinate to map coordinate
    // for(size_t i=0; i<(int)img_0->keypoint_vector_.size(); i++){

    //     std::shared_ptr<KeyPoint> &kp = img_0->keypoint_vector_.at(i);


    //     if(kp != nullptr && kp->prev_keypoint_in_time_ == nullptr && kp->pt3d_.z > 0){

    //         Eigen::Vector3d map_point = img_0->T_c_w_.inverse() * Eigen::Vector3d(kp->pt3d_.x, kp->pt3d_.y, kp->pt3d_.z);
    //         std::shared_ptr<MapPoint> map_point_ptr = std::make_shared<MapPoint>(map_point);
    //         cv::Vec3b bgr = img_0->color_data_.at<cv::Vec3b>(kp->cv_keypoint_.pt);
    //         map_point_ptr->setColor(bgr[0], bgr[1], bgr[2]);

    //         kp->setMapPointPtr(map_point_ptr);
    //         frame->map_point_vector_.emplace_back(map_point_ptr);
            
    //     }
        

    // }

    // if(frame->is_key_frame_){

    //     cv::Mat cv_K = this->sys_config_->camera_config_->getParamsAt<CameraParameters>(img_0->sensor_id_)->getCVIntrinsicsMatrix();

        
    //     for (int row = 0; row < img_0->depth_.rows; ++row) {
    //         for (int col = 0; col < img_0->depth_.cols; ++col) {

    //             double depth = img_0->depth_.at<double>(row, col);
    //             if(depth <= 0){
    //                 // LOG(INFO) << "depth = -1 at" << " row: " << row << ", col: " << col;
    //                 continue;
    //             }

    //             cv::Point2f pt2f = cv::Point2f(col, row);
    //             auto it = std::find_if(img_0->keypoint_vector_.begin(), img_0->keypoint_vector_.end(), 
    //             [&](const std::shared_ptr<KeyPoint> &kp){
    //                     return kp->pt2i_ == pt2i;
    //             });

                

    //             // skip all the keypoints, because their mappoints have created
    //             if(it != img_0->keypoint_vector_.end()){
    //                 continue;
    //             }

    //             cv::Point3d pt3d = pixel2camera(pt2i, depth, cv_K);
    //             // Eigen::Vector3d map_point;
    //             // map_point.x() = pt3d.x;
    //             // map_point.y() = pt3d.y;
    //             // map_point.z() = pt3d.z;
    //             Eigen::Vector3d map_point = img_0->T_c_w_.inverse() * Eigen::Vector3d(pt3d.x, pt3d.y, pt3d.z);
    //             std::shared_ptr<MapPoint> map_point_ptr = std::make_shared<MapPoint>(map_point);            
    //             cv::Vec3b bgr = img_0->color_data_.at<cv::Vec3b>(pt2i);
    //             map_point_ptr->setColor(bgr[0], bgr[1], bgr[2]);
    //             frame->map_point_vector_.emplace_back(map_point_ptr);
    //             // LOG(INFO) << "pt2i : " << pt2i << " depth : "  << depth << " color : " << bgr;
    //             // LOG(INFO) << "pt3d : " << pt3d ;
    //         }
    //     }

    // }
    

    int count_new_mappoint = 0;

    for(size_t i=0; i<frame->image_vector_.size();i++){
        
            const std::shared_ptr<Image> &img_i = frame->image_vector_.at(i);
            
            for(size_t j=0;j<img_i->keypoint_vector_.size();j++){
                
                const std::shared_ptr<MapPoint> &mp =  img_i->keypoint_vector_.at(j)->map_point_ptr_;
                if(mp != nullptr && !this->isExistedMapPoint(mp)){
                    this->insertMapPoint(mp);
                    count_new_mappoint++;
                }
            }
    }

    for(size_t i=0; i < frame->image_vector_.size();i++){
        
        const std::shared_ptr<Image> &img_i = frame->image_vector_.at(i);

        for(size_t j=0;j<img_i->mappoint_vector_.size();j++){
                
                const std::shared_ptr<MapPoint> &mp =  img_i->mappoint_vector_.at(j);
                if(mp != nullptr && !this->isExistedMapPoint(mp)){
                    this->insertMapPoint(mp);
                    count_new_mappoint++;
                }
            }

    }

    LOG(INFO) << GREEN << count_new_mappoint << " map points were added to local map" << RESET;
    LOG(INFO) << GREEN << "There are " << mappoints_.size() << " number of map point in total" << RESET;

    

    maintainSize();
    
}


void Map::maintainSize(){
    
    int mappoint_size = this->mappoints_.size();
    int oversize = mappoint_size - this->sys_config_->params_->max_num_local_map_size_;
    if(oversize > 0){
        auto it = this->mappoints_.begin();
        for (int i = 0; i < oversize && it != this->mappoints_.end(); ++i) {
            it = this->mappoints_.erase(it);  // erase returns the next iterator
        }
    }
}

    
} // namespace modules_vins




