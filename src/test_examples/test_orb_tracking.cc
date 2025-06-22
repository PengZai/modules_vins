#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <string>
#include "../system/system_config.h"
#include "../detect/detector.h"
#include "../tracking/descriptor_tracker.h"
#include "../data/camera.h"
#include "../data/preprocess.h"



using namespace modules_vins;

int main(int argc, char* argv[]) {

    std::string log_verbosity = "VERBOSE";
    modules_vins::Logger::setLogger(argv, log_verbosity);

    std::string config_path = "/root/catkin_ws/src/modules_vins/configs/BotanicGarden/system_config.yaml";
    std::shared_ptr<modules_vins::SystemConfig> sys_config = std::make_shared<modules_vins::SystemConfig>();
    sys_config->loadFromPath(config_path);

    std::shared_ptr<modules_vins::FeaturePoint> feature_point = std::make_shared<modules_vins::ORBFeature>(sys_config);
    std::shared_ptr<modules_vins::Detector> detector = std::make_shared<modules_vins::Detector>(sys_config);
    std::shared_ptr<modules_vins::DataPreprocesor> data_preprocesor = std::make_shared<modules_vins::DataPreprocesor>();

    detector->setFeaturePoint(feature_point);
    std::shared_ptr<modules_vins::Tracker> tracker = std::make_shared<modules_vins::DescriptorTracker>(sys_config);

    // Load two grayscale images
    cv::Mat img1 = cv::imread("/root/datasets2/BotanicGarden/1018-00/1018_00_img10hz600p/left_rgb/" + std::string(argv[1]), cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread("/root/datasets2/BotanicGarden/1018-00/1018_00_img10hz600p/left_rgb/" + std::string(argv[2]), cv::IMREAD_COLOR);

    if (img1.empty() || img2.empty()) {
        std::cerr << "Failed to load images!" << std::endl;
        return -1;
    }

    std::vector<std::shared_ptr<modules_vins::Image>> image_vector_1, image_vector_2;
    std::shared_ptr<Image> img_1 = std::make_shared<Image>(1666059843550117970, 0, img1.clone());
    std::shared_ptr<Image> img_2 = std::make_shared<Image>(1666059843650118113, 0, img2.clone());
    image_vector_1.push_back(img_1);
    image_vector_2.push_back(img_2);

    std::shared_ptr<CameraFrame>camera_frame_1 = std::make_shared<CameraFrame>(image_vector_1);
    std::shared_ptr<CameraFrame>camera_frame_2 = std::make_shared<CameraFrame>(image_vector_2);

    camera_frame_1->status_ = CameraFrame::Status::NORMAL;
    camera_frame_2->status_ = CameraFrame::Status::NORMAL;
    camera_frame_2->ref_camera_frame_ = camera_frame_1;

    data_preprocesor->pipeline(camera_frame_1);
    data_preprocesor->pipeline(camera_frame_2);

    detector->pipeline(camera_frame_1);
    detector->pipeline(camera_frame_2);

    cv::Mat descriptor1, descriptor2;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;

    img_1->getDescripots(descriptor1);
    img_2->getDescripots(descriptor2);
    img_1->getCVKeyPoints(keypoints1);
    img_2->getCVKeyPoints(keypoints2);

    tracker->pipeline(camera_frame_2);
    // std::vector<cv::DMatch> &matches  = camera_frame_1->image_vector_.at(0)->matches_in_time_;

    std::vector<cv::DMatch> matches;
    // Step 2: Match descriptors using brute-force matcher with Hamming distance
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);  // crossCheck=true
    matcher.match(descriptor1, descriptor2, matches);

    // Step 3: Filter good matches based on distance
    double max_dist = 0; double min_dist = 100;
    for (const auto& m : matches) {
        double dist = m.distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    std::vector<cv::DMatch> good_matches;
    for (const auto& m : matches) {
        if (m.distance <= std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(m);
        }
    }

    // Step 4: Draw matches
    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches);

    // Show the result
    cv::imshow("ORB Feature Tracking", img_matches);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}
