#include "state.h"

namespace modules_vins{




State::State(const std::shared_ptr<SystemConfig> sys_config):
sys_config_(sys_config)
{

    for(size_t i=0;i<this->sys_config_->comparison_config_->params_vector_.size();i++){

        const std::shared_ptr<ComparisonParameters> &comparison_params = this->sys_config_->comparison_config_->params_vector_.at(i);
        Eigen::Matrix<double, 4, 4> R_cam_GT = comparison_params->T_cam_comparison_; //Rotation only
        R_cam_GT.block<3,1>(0, 3).setZero(); 
        this->R_cam_comparison_vector_.emplace_back(Sophus::SE3d::fitToSE3(R_cam_GT));

    }
}

bool State::SynchronizeAndTransformComparisonPoseToTcw(const double base_timestamp){


    for(size_t idx=0; idx < this->sys_config_->comparison_config_->params_vector_.size(); idx++){

        const std::shared_ptr<ComparisonParameters> &comparison_params = this->sys_config_->comparison_config_->params_vector_.at(idx);
        const std::string &comparison_name = comparison_params->name_;
        const std::map<double, Sophus::SE3<double>> &timestamp_comparison_T_w_c_full_map_ = this->timestamp_comparison_T_w_c_full_map_vector_.at(idx);


        double synchronized_gt_timestamp = getSynchronizedPoseTimestamp(base_timestamp, comparison_params->max_tolerant_time_offset_, timestamp_comparison_T_w_c_full_map_);
        if(synchronized_gt_timestamp == -1){
            return false;
        }

        Sophus::SE3<double> system_initialized_first_GT_T_w_inv = timestamp_comparison_T_w_c_full_map_.at(synchronized_gt_timestamp).inverse();

        auto it_start = timestamp_comparison_T_w_c_full_map_.find(synchronized_gt_timestamp);
        auto it_end = timestamp_comparison_T_w_c_full_map_.end();

        std::map<double, Sophus::SE3d> comparison_T_sub_map(it_start, it_end);

        Sophus::SE3d coordinate_transformation;

        if(comparison_params->set_first_pose_in_origin_){
            // coordinate_transformation = this->R_cam_comparison_vector_.at(idx) * system_initialized_first_GT_T_w_inv;
            coordinate_transformation = this->R_cam_comparison_vector_.at(idx);

        }
        else{
            coordinate_transformation = this->R_cam_comparison_vector_.at(idx);
        }


        std::map<double, Sophus::SE3<double>> timestamp_comparison_T_c_w_sub_map_;
        for (const auto& [timestamp, comparison_T_w_c] : timestamp_comparison_T_w_c_full_map_) {
                timestamp_comparison_T_c_w_sub_map_[timestamp] =  (coordinate_transformation * comparison_T_w_c).inverse();    
                // Eigen::Vector4d new_translation;
                // new_translation << comparison_T_w_c.translation(), 1.0;

                // new_translation = coordinate_transformation * new_translation;

          
                // timestamp_comparison_T_c_w_sub_map_[timestamp] =  Sophus::SE3d(
                //                                 Sophus::SO3d(comparison_T_w_c.rotationMatrix()), 
                //                                 Eigen::Vector3d(new_translation(0),new_translation(1),new_translation(2))
                //                                 ).inverse();    

        }

        this->timestamp_comparison_T_c_w_map_vector_.emplace_back(timestamp_comparison_T_c_w_sub_map_);

    }
    
    
    return true;

}


bool State::getCameraPoseWithComparisonPoseIdx(const double base_timestamp, size_t used_idx, Sophus::SE3d &T_c_w){
    

    const std::shared_ptr<ComparisonParameters> &comparison_param = this->sys_config_->comparison_config_->params_vector_.at(used_idx);


    std::map<double, Sophus::SE3<double>> timestamp_comparison_T_c_w_map = this->timestamp_comparison_T_c_w_map_vector_.at(used_idx);
    double synchronized_gt_timestamp = getSynchronizedPoseTimestamp(base_timestamp, comparison_param->max_tolerant_time_offset_, timestamp_comparison_T_c_w_map);
    double time_difference = synchronized_gt_timestamp - base_timestamp;
    if(synchronized_gt_timestamp == -1){
        return false;
    }

    Sophus::SE3<double> T_GT_w_ = timestamp_comparison_T_c_w_map.at(synchronized_gt_timestamp);

    T_c_w = T_GT_w_;

    return true;
}

double State::getSynchronizedPoseTimestamp(const double base_timestamp, const double max_tolerant_time_offset, std::map<double, Sophus::SE3<double>> timestamp_comparison_T_w_c_full_map) const{

    double smallest_time_offset = std::numeric_limits<double>::infinity();
    double smallest_time_offset_timestamp = -1;
    double synchronized_timestamp = -1;

    for (const auto& [timestamp, GT_T_w] : timestamp_comparison_T_w_c_full_map) {


        double time_offset = std::abs(base_timestamp-timestamp);
        if(time_offset < smallest_time_offset){
            smallest_time_offset = time_offset;
            smallest_time_offset_timestamp = timestamp;
        }
        // if tiem_offset in next index is greather than previous one then we don't have to look at the rest index anymore
        // because msgs are sorted by chronological order. so here we break the loop directly
        else{
            break;
        }
    }

    if(smallest_time_offset < max_tolerant_time_offset){
        synchronized_timestamp = smallest_time_offset_timestamp; 
    }

    return synchronized_timestamp;
}


} //namespace modules_vins