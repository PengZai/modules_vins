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



void Map::update(const std::shared_ptr<CameraFrame> &camera_frame){

    if(camera_frame->status_ != CameraFrame::Status::NORMAL){
        return;
    }

    if(camera_frame->image_vector_.size()==0){

        VLOG(VERBOSE) << RED << "size of image vector equal to 0" << RESET;
        std::exit(EXIT_FAILURE);
    }

    int count_new_mappoint = 0;

    for(int i=0; i<(int)camera_frame->map_point_vector_.size();i++){
        
            const std::shared_ptr<MapPoint> &mp = camera_frame->map_point_vector_.at(i);
            
            if(!this->isExistedMapPoint(mp)){
                this->insertMapPoint(mp);
                count_new_mappoint++;
            }
        

    }

    VLOG(VERBOSE) << GREEN << count_new_mappoint << " map points were added to local map" << RESET;

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




