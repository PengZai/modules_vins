#include "utils.h"


namespace modules_vins{

// K = {
// fx,0,cx,
// 0,fy,cy,
// 0, 0, 1
// }



// convert keypoint from pixel image plane to normalized camera plane
cv::Point2d pixel2norm(const cv::Point2d &pt2d, const cv::Mat &K ){   

    cv::Point2d noramlized_pt2d = cv::Point2d(
        ( pt2d.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( pt2d.y - K.at<double>(1,2) ) / K.at<double>(1,1)  
    );


    return noramlized_pt2d;

}




// convert keypoint from pixel image plane to camera coordinate
cv::Point3d pixel2camera(const cv::Point2d &pt2d, double depth, cv::Mat &K)
{
    return cv::Point3d (
        ( pt2d.x - K.at<double>(0,2) ) *depth / K.at<double>(0,0), 
        ( pt2d.y - K.at<double>(1,2) ) *depth / K.at<double>(1,1),
        depth
    );
}


// convert keypoint from pixel image plane to camera coordinate
cv::Point2d camera2pixel(const cv::Point3d &pt3d, cv::Mat &K)
{
    return cv::Point2d(
        ( pt3d.x * K.at<double>(0,0) ) / pt3d.z + K.at<double>(0,2), 
        ( pt3d.y * K.at<double>(1,1) ) / pt3d.z + K.at<double>(1,2)
    );
}





} //namespace modules_vins
