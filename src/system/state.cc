#include "state.h"

namespace modules_vins{




State::State(const std::shared_ptr<SystemConfig> sys_config):
sys_config_(sys_config)
{

    for(size_t i=0;i<this->sys_config_->comparison_config_->params_vector_.size();i++){

        const std::shared_ptr<ComparisonParameters> &comparison_params = this->sys_config_->comparison_config_->getParamsAt<ComparisonParameters>(i);
        Eigen::Matrix<double, 4, 4> T_base_comparison = comparison_params->T_base_sensor_; //Rotation only
        T_base_comparison.block<3,1>(0, 3).setZero(); 
        this->T_base_comparison_vector_.emplace_back(Sophus::SE3d::fitToSE3(T_base_comparison));

    }
}

bool State::SynchronizeAndTransformComparisonPoseToRobotBaseCoordinate(const double base_timestamp){


    for(size_t idx=0; idx < this->sys_config_->comparison_config_->params_vector_.size(); idx++){

        const std::shared_ptr<ComparisonParameters> &comparison_params = this->sys_config_->comparison_config_->getParamsAt<ComparisonParameters>(idx);
        const std::string &comparison_name = comparison_params->name_;
        const std::map<double, Sophus::SE3<double>> &timestamp_pose_comparison_in_comparison_full_map = this->timestamp_pose_comparison_in_comparison_full_map_vector_.at(idx);


        double synchronized_gt_timestamp = getSynchronizedPoseTimestamp(base_timestamp, comparison_params->max_tolerant_time_offset_, timestamp_pose_comparison_in_comparison_full_map);
        if(synchronized_gt_timestamp == -1){
            return false;
        }

        // Sophus::SE3<double> system_initialized_first_comparison_pose_inv = timestamp_pose_comparison_in_comparison_full_map.at(synchronized_gt_timestamp).inverse();

        auto it_start = timestamp_pose_comparison_in_comparison_full_map.find(synchronized_gt_timestamp);
        auto it_end = timestamp_pose_comparison_in_comparison_full_map.end();

        std::map<double, Sophus::SE3d> timestamp_pose_comparison_in_comparison_sub_map(it_start, it_end);

        Sophus::SE3d T_base_comparison;

        if(comparison_params->set_first_pose_in_origin_){
            // coordinate_transformation = this->T_imu0_comparison_vector_.at(idx) * system_initialized_first_GT_T_w_comp_inv;
            T_base_comparison = this->T_base_comparison_vector_.at(idx);

        }
        else{
            T_base_comparison = this->T_base_comparison_vector_.at(idx);
        }


        std::map<double, Sophus::SE3<double>> timestamp_pose_comparison_in_base_map;
        for (const auto& [timestamp, comparison_pose_in_comparison] : timestamp_pose_comparison_in_comparison_sub_map) {
                timestamp_pose_comparison_in_base_map[timestamp] =  (T_base_comparison * comparison_pose_in_comparison).inverse();

        }

        this->timestamp_pose_comparison_in_base_map_vector_.emplace_back(timestamp_pose_comparison_in_base_map);

    }
    
    
    return true;

}


bool State::getPoseComparisonForTbwWithIdx(const double base_timestamp, size_t used_idx, Sophus::SE3d &T_b_w){
    

    const std::shared_ptr<ComparisonParameters> &comparison_param = this->sys_config_->comparison_config_->getParamsAt<ComparisonParameters>(used_idx);


    std::map<double, Sophus::SE3<double>> timestamp_pose_comparison_in_base_map = this->timestamp_pose_comparison_in_base_map_vector_.at(used_idx);
    double synchronized_pose_comparison_in_base_timestamp = getSynchronizedPoseTimestamp(base_timestamp, comparison_param->max_tolerant_time_offset_, timestamp_pose_comparison_in_base_map);
    if(synchronized_pose_comparison_in_base_timestamp == -1){
        return false;
    }
    double time_difference = synchronized_pose_comparison_in_base_timestamp - base_timestamp;


    Sophus::SE3<double> pose_comparison_in_base = timestamp_pose_comparison_in_base_map.at(synchronized_pose_comparison_in_base_timestamp);

    T_b_w = pose_comparison_in_base;

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