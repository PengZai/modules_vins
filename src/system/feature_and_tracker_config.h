#include "../log/logging.h"

#include <memory>


#include "config.h"

namespace modules_vins{


class ORBParameters: public Parameters{

    public:
    void loadFromNode(const std::shared_ptr<cv::FileNode> &node) override;

    public:
    int num_feature_points_;
    int block_size_;
    int fastThreshold_;
    bool useNonmaxSuppression_;
    // double scale_factor_;
    // int level_pyramid_;
    double threshold_for_tracking_descriptor_in_time_;
    double threshold_for_tracking_descriptor_in_frame_;
    double matching_ratio_; // distance of matching point < min_distance * matching_ratio
};



class KLTParameters: public Parameters{

    public:
    void loadFromNode(const std::shared_ptr<cv::FileNode> &node) override;

    public:
    int num_feature_points_;
    int min_distance_; // radius size for image block only containing one feature point, so that we can distribute feature points on (image size/ min_distance * 2) image block
    int max_count_;	// Max number of iterations (used if COUNT is set)
    double epsilon_; // Min required accuracy / threshold (used if EPS is set)

};


class FeatureAndTrackerParameters : public Parameters{


    public:
    void loadFromNode(const std::shared_ptr<cv::FileNode> &node) override;

    public:

    int place_holder_;

};

class FeatureAndTrackerConfig : public Config{


    
    public:

    std::shared_ptr<FeatureAndTrackerParameters> params_;
    std::shared_ptr<ORBParameters> orb_params_;
    std::shared_ptr<KLTParameters> klt_params_;




};
    


} //modules_vins



