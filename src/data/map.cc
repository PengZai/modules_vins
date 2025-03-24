#include "map.h"


namespace modules_vins
{


Map::Map()
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



void Map::update(const CameraFrame &camera_frame){

    if(camera_frame.image_vector_.size()==0){

        VLOG(VERBOSE) << RED << "size of image vector equal to 0" << RESET;
        std::exit(EXIT_FAILURE);
    }

    for(int i=0; i<(int)camera_frame.image_vector_.at(0)->keypoint_vector_.size();i++){

        const std::shared_ptr<KeyPoint> &kp = camera_frame.image_vector_.at(0)->keypoint_vector_.at(i);
        
        if(kp->map_point_ptr_ != nullptr){

            const std::shared_ptr<MapPoint> &mp = kp->map_point_ptr_;
            
            if(!this->isExistedMapPoint(mp)){
                this->insertMapPoint(mp);
            }

        }

    }
    
}

    
} // namespace modules_vins




