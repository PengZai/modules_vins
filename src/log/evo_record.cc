#include "evo_record.h"


namespace modules_vins{


EVORecorder::EVORecorder(const std::shared_ptr<SystemConfig> sys_config):
sys_config_(sys_config)
{   

    boost::filesystem::path output_dir(this->sys_config_->params_->output_dir_);
    if (boost::filesystem::create_directories(output_dir)) {
        VLOG(VERBOSE) << "create a folder " << output_dir<< " path for output file";
    }

    
    boost::filesystem::path traj_filename = "estimated_traj.txt";
    boost::filesystem::path traj_path = output_dir / traj_filename;
    // If it exists, then delete it
    if (boost::filesystem::exists(traj_path)) {
        VLOG(VERBOSE) << "Output file exists, deleting old file....";
        boost::filesystem::remove(traj_path);
    }
    
    this->outfile_.open(traj_path.string());
    if(this->outfile_.fail()){
        VLOG(VERBOSE) << RED << "Unable to open output file!!" << RESET;
    }

    this->outfile_ << "# timestamp(s) tx ty tz qx qy qz qw " << std::endl;

}

void EVORecorder::writeTrajectoryOnce(const double timestamp, const Eigen::Vector3d &t, const Eigen::Quaterniond &q){
    
    // timestamp
    this->outfile_ << std::to_string(timestamp) << " ";

    //pose
    this->outfile_.precision(6);
    this->outfile_ << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

    this->outfile_ << std::endl;

}



} // namespace modules_vins