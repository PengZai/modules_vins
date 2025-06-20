#include "ros1_visualizer.h"



namespace modules_vins{

    

ROS1Visualizer::ROS1Visualizer(const std::shared_ptr<SystemConfig> &sys_config, const std::shared_ptr<ros::NodeHandle> &nh):
sys_config_(sys_config), 
nh_(nh), 
it_(*nh), 
ros_rate_(40)

{

    for(size_t i=0; i < this->sys_config_->params_->max_cameras_; i++){
        this->output_image_pub_vector_.push_back(this->it_.advertise(this->sys_config_->camera_config_->params_vector_.at(i)->output_rostopic_, 1));
    }


    this->tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

    this->output_pose_pub_ = this->nh_->advertise<geometry_msgs::PoseStamped>(this->sys_config_->visualizer_config_->rviz_params_->output_pose_rostopic_, 1);
    this->output_key_frame_poses_pub_ = this->nh_->advertise<geometry_msgs::PoseArray>(this->sys_config_->visualizer_config_->rviz_params_->output_key_frame_poses_rostopic_, 1);
    this->output_GT_pose_pub_ = this->nh_->advertise<geometry_msgs::PoseStamped>(this->sys_config_->visualizer_config_->rviz_params_->output_groundtruth_pose_rostopic_, 1);
    this->output_trajectory_pub_ = this->nh_->advertise<nav_msgs::Path>(this->sys_config_->visualizer_config_->rviz_params_->output_trajectory_rostopic_, 1);
    this->output_GT_trajectory_pub_ = this->nh_->advertise<nav_msgs::Path>(this->sys_config_->visualizer_config_->rviz_params_->output_groundtruth_trajectory_rostopic_, 1);
    this->output_tracked_map_points_pub_ = this->nh_->advertise<sensor_msgs::PointCloud2>(this->sys_config_->visualizer_config_->rviz_params_->output_tracked_map_points_rostopic_, 1);

    // just for test
    this->R_.setIdentity();  // Rotation matrix
    this->t_.setZero();  // Translation vector

    pose_frame_id_ = "cam0";

    nav_msgs::Path path_msg_;
    

}

void ROS1Visualizer::setNodehandler(const std::shared_ptr<ros::NodeHandle> &nh){
    this->nh_ = nh;
}

void ROS1Visualizer::setMap(const std::shared_ptr<Map> &map){
    this->map_ = map;
}

void ROS1Visualizer::publish(const std::shared_ptr<CameraFrame> &camera_frame, const State &state){
    


    if(ros::ok()){
        publishTF();
        publishImages(camera_frame);
        publishPoses(state);
        publishKeyPoses(state);
        publishTrajectories(state);
        publishMapPoint(state);


        // ros::spinOnce();
        // this->ros_rate_.sleep();
    }



}

void ROS1Visualizer::publishImages(const std::shared_ptr<CameraFrame> &camera_frame){

    std_msgs::Header header;

    for(size_t i=0; i < this->sys_config_->params_->max_cameras_; i++){
        const std::shared_ptr<Image> &img = camera_frame->image_vector_.at(i);
        header.stamp = ros::Time::now();
        header.frame_id = "cam" + img->sensor_id_;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img->color_data_).toImageMsg();
        output_image_pub_vector_.at(i).publish(msg);
    
    }


}

void ROS1Visualizer::publishTF(){
    tf::StampedTransform global2cam0_trans;
    global2cam0_trans.stamp_ = ros::Time::now();
    global2cam0_trans.frame_id_ = "global";
    global2cam0_trans.child_frame_id_ = "cam0";
    tf::Quaternion global2cam0_quaternion;
    // In ROS, use BGR-ZYX (yaw-pitch-roll) order. 
    // this according to matrix
    // 0,1,0
    // 0,0,-1
    // -1,0,0
    global2cam0_quaternion.setRPY(-M_PI/2, 0, M_PI/2);
    global2cam0_trans.setRotation(global2cam0_quaternion);

    tf::Vector3 global2cam0_origin(-1, 0, 0);
    global2cam0_trans.setOrigin(global2cam0_origin);

    this->tf_broadcaster_->sendTransform(global2cam0_trans);

    tf::StampedTransform global2imu_trans;
    global2imu_trans.stamp_ = ros::Time::now();
    global2imu_trans.frame_id_ = "global";
    global2imu_trans.child_frame_id_ = "imu";
    tf::Quaternion global2imu_quaternion;
    // In ROS, use BGR-ZYX (yaw-pitch-roll) order. 
    global2imu_quaternion.setRPY(0, 0, 0);
    global2imu_trans.setRotation(global2imu_quaternion);
    tf::Vector3 global2imu_origin(0, 1, 0);
    global2imu_trans.setOrigin(global2imu_origin);

    this->tf_broadcaster_->sendTransform(global2imu_trans);


}


void ROS1Visualizer::constructPoseMsg(const Sophus::SE3<double> &pose, geometry_msgs::PoseStamped &pose_msg){

    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = this->pose_frame_id_;

    const Eigen::Matrix3d &rotation = pose.rotationMatrix();
    const Eigen::Vector3d &position = pose.translation();

    // Example translation and rotation
    const Eigen::Quaterniond q(rotation);

    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
 

}


void ROS1Visualizer::constructPoseMsg(const Sophus::SE3<double> &pose, geometry_msgs::Pose &pose_msg){


    const Eigen::Matrix3d &rotation = pose.rotationMatrix();
    const Eigen::Vector3d &position = pose.translation();

    // Example translation and rotation
    const Eigen::Quaterniond q(rotation);

    pose_msg.position.x = position.x();
    pose_msg.position.y = position.y();
    pose_msg.position.z = position.z();

    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
 

}


void ROS1Visualizer::publishPoses(const State &state){

    // const Sophus::SE3<double> &T_c_w = state.T_c_w_vector_.back().inverse(); // the vector of pose of robot in world coordinate
    if(!state.timestamp_T_c_w_map_.empty()){
        auto it = state.timestamp_T_c_w_map_.rbegin();
        const double newest_timestamp = it->first;
        const Sophus::SE3<double> &newest_T_c_w = it->second;

        geometry_msgs::PoseStamped pose_msg;
        constructPoseMsg(newest_T_c_w.inverse(), pose_msg);
        this->output_pose_pub_.publish(pose_msg);


        if(this->sys_config_->visualizer_config_->rviz_params_->show_groundtruth_pose_){

            double synchronized_gt_timestamp = state.findSynchronizedPoseTimestamp(newest_timestamp, this->sys_config_->params_->max_tolerant_gt_time_offset_);
            if(synchronized_gt_timestamp == -1){
                return;
            }
            const Sophus::SE3<double> &synchronized_GT_T_c_w = state.timestamp_GT_T_c_w_map_.at(synchronized_gt_timestamp);

            constructPoseMsg(synchronized_GT_T_c_w, pose_msg);
            this->output_GT_pose_pub_.publish(pose_msg);

        }
    }
    
    

}


void ROS1Visualizer::publishKeyPoses(const State &state){

    if(!state.timestamp_key_T_c_w_map_.empty()){
        
        geometry_msgs::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = ros::Time::now();
        pose_array_msg.header.frame_id = this->pose_frame_id_;

        for (const auto& [timestamp, T_c_w] : state.timestamp_key_T_c_w_map_) {
            
            geometry_msgs::Pose pose_msg;
            constructPoseMsg(T_c_w.inverse(), pose_msg);
            pose_array_msg.poses.push_back(pose_msg);
        }

        this->output_key_frame_poses_pub_.publish(pose_array_msg);



    }
}

void ROS1Visualizer::publishTrajectories(const State &state){

    nav_msgs::Path path_msgs;
    if(!state.timestamp_T_c_w_map_.empty()){
        publishTrajectory(state.timestamp_T_c_w_map_, path_msgs, this->output_trajectory_pub_);
        if(this->sys_config_->visualizer_config_->rviz_params_->show_groundtruth_trajectory_){

            auto it = state.timestamp_T_c_w_map_.rbegin();
            const double newest_timestamp = it->first;
            double synchronized_gt_timestamp = state.findSynchronizedPoseTimestamp(newest_timestamp, this->sys_config_->params_->max_tolerant_gt_time_offset_);
            if(synchronized_gt_timestamp == -1){
                return;
            }

            auto it_end = state.timestamp_GT_T_c_w_map_.find(synchronized_gt_timestamp);

            std::map<double, Sophus::SE3<double>> timestamp_GT_T_c_w_sub_map(state.timestamp_GT_T_c_w_map_.begin(), it_end);

            publishGTTrajectory(timestamp_GT_T_c_w_sub_map, path_msgs, this->output_GT_trajectory_pub_);
        }
    }
    

}



void ROS1Visualizer::publishTrajectory(const std::map<double, Sophus::SE3<double>> &timestamp_T_c_w_map, nav_msgs::Path &path_msgs, ros::Publisher output_trajectory_pub){

    path_msgs.poses.clear();
    path_msgs.header.stamp = ros::Time::now();
    path_msgs.header.frame_id = this->pose_frame_id_;

    for (const auto& [timestamp, T_c_w] : timestamp_T_c_w_map) {

        geometry_msgs::PoseStamped pose_msg;
        constructPoseMsg(T_c_w.inverse(), pose_msg);


        path_msgs.poses.push_back(pose_msg);
    }

    output_trajectory_pub.publish(path_msgs);

}

void ROS1Visualizer::publishGTTrajectory(const std::map<double, Sophus::SE3<double>> &timestamp_T_c_w_map, nav_msgs::Path &path_msgs, ros::Publisher output_trajectory_pub){

    path_msgs.poses.clear();
    path_msgs.header.stamp = ros::Time::now();
    path_msgs.header.frame_id = this->pose_frame_id_;

    for (const auto& [timestamp, T_c_w] : timestamp_T_c_w_map) {

        geometry_msgs::PoseStamped pose_msg;
        constructPoseMsg(T_c_w, pose_msg);


        path_msgs.poses.push_back(pose_msg);
    }

    output_trajectory_pub.publish(path_msgs);

}


void ROS1Visualizer::publishMapPoint(const State &state){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_point_collection_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    const std::map<unsigned int, std::shared_ptr<MapPoint>>& map_points = state.map_->getMapPoints();

    for(const std::pair<const unsigned int, std::shared_ptr<MapPoint>> &item_pair: map_points){
        pcl::PointXYZRGB rgb_map_point;
        const std::shared_ptr<MapPoint> &map_point = item_pair.second;

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
     map_point_msgs.header.frame_id = this->pose_frame_id_;


     this->output_tracked_map_points_pub_.publish(map_point_msgs);



}



    

} //modules_vins