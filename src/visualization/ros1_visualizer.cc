#include "ros1_visualizer.h"



namespace modules_vins{

    

ROS1Visualizer::ROS1Visualizer(const std::shared_ptr<SystemConfig> &sys_config, const std::shared_ptr<ros::NodeHandle> &nh):
sys_config_(sys_config), 
nh_(nh), 
it_(*nh), 
ros_rate_(40)

{

    for(int i=0; i < this->sys_config_->params_->max_cameras_; i++){
        this->output_image_pub_vector_.push_back(this->it_.advertise(this->sys_config_->camera_config_->params_vector_.at(i)->output_rostopic_, 1));
    }



    this->output_pose_pub_ = this->nh_->advertise<geometry_msgs::PoseStamped>(this->sys_config_->visualizer_config_->rviz_params_->output_pose_rostopic_, 1);
    this->output_trajectory_pub_ = this->nh_->advertise<nav_msgs::Path>(this->sys_config_->visualizer_config_->rviz_params_->output_trajectory_rostopic_, 1);
    this->output_tracked_map_points_pub_ = this->nh_->advertise<sensor_msgs::PointCloud2>(this->sys_config_->visualizer_config_->rviz_params_->output_tracked_map_points_rostopic_, 1);

    // just for test
    this->R_.setIdentity();  // Rotation matrix
    this->t_.setZero();  // Translation vector

    nav_msgs::Path path_msg_;
    

}

void ROS1Visualizer::setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh){
    this->nh_ = nh;
}

void ROS1Visualizer::publish(const CameraFrame &camera_frame){
    


    if(ros::ok()){
        publishImages(camera_frame);
        publishPoses(camera_frame);
        publishTrajectory(camera_frame);
        publishMapPoint(camera_frame);


        // ros::spinOnce();
        // this->ros_rate_.sleep();
    }



}

void ROS1Visualizer::publishImages(const CameraFrame &camera_frame){

    std_msgs::Header header;

    for(int i=0; i < this->sys_config_->params_->max_cameras_; i++){
        const std::shared_ptr<Image> &img = camera_frame.image_vector_.at(i);
        header.stamp = ros::Time::now();
        header.frame_id = "cam" + img->sensor_id_;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img->color_data_).toImageMsg();
        output_image_pub_vector_.at(i).publish(msg);
    
    }


}


void ROS1Visualizer::publishPoses(const CameraFrame &camera_frame){

    const std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";

    Eigen::Matrix3d rotation = img_0->getRotation();
    Eigen::Vector3d position = img_0->getPosition();

    // Example translation and rotation
    Eigen::Quaterniond q(rotation);

    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    this->output_pose_pub_.publish(pose_msg);

}


void ROS1Visualizer::publishTrajectory(const CameraFrame &camera_frame){

    const std::shared_ptr<Image> &img_0 = camera_frame.image_vector_.at(0);
    
    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = "map";

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";

    Eigen::Matrix3d rotation = img_0->getRotation();
    Eigen::Vector3d position = img_0->getPosition();

    Eigen::Quaterniond q(rotation);

    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    this->path_msg_.poses.push_back(pose_msg);

    this->output_trajectory_pub_.publish(this->path_msg_);

}


void ROS1Visualizer::publishMapPoint(const CameraFrame &camera_frame){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_point_collection_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::shared_ptr<Image> img_0 = camera_frame.image_vector_.at(0);

    cv::Mat cv_K = this->sys_config_->camera_config_->params_vector_.at(img_0->sensor_id_)->getCVIntrinsicsMatrix();

    const std::map<unsigned int, std::shared_ptr<MapPoint>>& map_points = camera_frame.getMap()->getMapPoints();

    for(const std::pair<const unsigned int, std::shared_ptr<MapPoint>> &item_pair: map_points){
        pcl::PointXYZRGB rgb_map_point;
        const std::shared_ptr<MapPoint> &map_point = item_pair.second;

 
        // const Eigen::Vector3d pt3d_in_cam = img_0->T_c_w_ * map_point->pt3d_;
        // cv::Point2d reprojected_pixel = camera2pixel(cv::Point3d(pt3d_in_cam.x(), pt3d_in_cam.y(), pt3d_in_cam.z()), cv_K);
        // if(img_0->isInImage(reprojected_pixel)){
        //     cv::Vec3b rgb = img_0->color_data_.at<cv::Vec3b>(reprojected_pixel);

        //     rgb_map_point.x = map_point->pt3d_[0]; // from depth
        //     rgb_map_point.y = map_point->pt3d_[1];
        //     rgb_map_point.z = map_point->pt3d_[2];
        
        //     rgb_map_point.r = rgb[0];  // from color image
        //     rgb_map_point.g = rgb[1];
        //     rgb_map_point.b = rgb[2];
        
        //     map_point_collection_ptr->points.push_back(rgb_map_point);

        // }

        rgb_map_point.x = map_point->pt3d_[0]; // from depth
        rgb_map_point.y = map_point->pt3d_[1];
        rgb_map_point.z = map_point->pt3d_[2];
    
        rgb_map_point.r = map_point->bgr_[2];  // from color image
        rgb_map_point.g = map_point->bgr_[1];
        rgb_map_point.b = map_point->bgr_[0];

        map_point_collection_ptr->points.push_back(rgb_map_point);

        
     }

     sensor_msgs::PointCloud2 map_point_msgs;
     pcl::toROSMsg(*map_point_collection_ptr, map_point_msgs);
     map_point_msgs.header.stamp = ros::Time::now();
     map_point_msgs.header.frame_id = "map";


     this->output_tracked_map_points_pub_.publish(map_point_msgs);



}



    

} //modules_vins