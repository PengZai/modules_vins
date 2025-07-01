#include "feature_and_tracker_config.h"


namespace modules_vins{




void ORBParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;

    parse("num_feature_points", this->num_feature_points_);
    parse("block_size", this->block_size_);
    parse("fastThreshold", this->fastThreshold_);
    parse("useNonmaxSuppression", this->useNonmaxSuppression_);

    // parse("scale_factor", this->scale_factor_);
    // parse("level_pyramid", this->level_pyramid_);
    parse("threshold_for_tracking_descriptor_in_time", this->threshold_for_tracking_descriptor_in_time_);
    parse("threshold_for_tracking_descriptor_in_frame", this->threshold_for_tracking_descriptor_in_frame_);
    parse("matching_ratio", this->matching_ratio_);


}


void KLTParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;

    parse("num_feature_points", this->num_feature_points_);
    parse("max_count", this->max_count_); // Min required accuracy / threshold (used if EPS is set)
    parse("epsilon", this->epsilon_); // Max number of iterations (used if COUNT is set)
    parse("min_distance", this->min_distance_);


}

void FeatureAndTrackerParameters::loadFromNode(const std::shared_ptr<cv::FileNode> &node){

    this->node_ = node;

    parse("place_holder", this->place_holder_);


}







} //modules_vins