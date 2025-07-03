#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "../log/logging.h"




namespace modules_vins
{   





class Parameters{
 
    public:

        virtual ~Parameters() = default;
        virtual void loadFromNode(const std::shared_ptr<cv::FileNode> &node) = 0;
        void setName(const std::string name);


        template <typename T>
        void parse(const std::string &parameter_name, T &parsed_value){
            (*this->node_)[parameter_name] >> parsed_value;
            LOG(INFO) << parameter_name << " : " << parsed_value;
        }

        template <typename T>
        void parse(const std::string &parameter_name, std::vector<T> &parsed_values){
            std::stringstream log_stream;

            (*this->node_)[parameter_name] >> parsed_values;

            log_stream << parameter_name << " : [ ";

            for (size_t i = 0; i < parsed_values.size(); ++i) {
                log_stream << parsed_values[i] << ", ";
            }
            
            LOG(INFO) << log_stream.str() << "]";
 
        }

       

        void parse(const std::string &parameter_name, Eigen::VectorXd &parsed_values){
            std::stringstream log_stream;
            
            cv::FileNode node = (*this->node_)[parameter_name];
            log_stream << parameter_name << " : [ ";

            for (size_t i = 0; i < node.size(); ++i) {
                parsed_values(i) = static_cast<double>(node[i]);
                log_stream << parsed_values(i) << ", ";
                
            }
            
            LOG(INFO) << log_stream.str() << "]";
 
        }

        void parse(const std::string &parameter_name, Eigen::Vector3d &parsed_values){
            std::stringstream log_stream;
            
            cv::FileNode node = (*this->node_)[parameter_name];
            
            if(node.size() > 3){
                LOG(INFO) << RED << "don't put more than 3 elements on a Vector3d" << RESET;
                std::exit(0);
                return;
            }

            log_stream << parameter_name << " : [ ";

            for (size_t i = 0; i < node.size(); ++i) {
                parsed_values(i) = static_cast<double>(node[i]);
                log_stream << parsed_values(i) << ", ";
                
            }
            
            LOG(INFO) << log_stream.str() << "]";
 
        }

        void parse(const std::string &parameter_name, Eigen::Matrix4d &parsed_values){
            std::stringstream log_stream;
            cv::FileNode node = (*this->node_)[parameter_name];
            
            log_stream << parameter_name << " : [ " << "\n";

            for (size_t i = 0; i < node.size(); ++i) {
                cv::FileNode row = node[i];
                if (row.size() != 4) {
                    std::cerr << "Error: Row " << i << " does not have 4 columns!" << std::endl;
                    return;
                }
        
                for (size_t j = 0; j < row.size(); ++j) {
                    parsed_values(i, j) = static_cast<double>(row[j]);

                    log_stream << parsed_values(i) << ", ";
                }

                if(i < node.size()-1) {log_stream << "\n";}
            }

            LOG(INFO) << log_stream.str() << "]";


        }


        



        std::shared_ptr<cv::FileNode> node_;
        Eigen::Matrix4d T_imu0_sensor_;
        Eigen::Matrix4d T_base_sensor_;
        std::string name_;

};


class Config
{
    public:


        void loadConfigFromPath(const std::string &config_path);

        template<typename ParameterType>
        void calculateExtrinsicsAndProjectionMatrixBetweenSensors(){

            for(size_t sensor_id_i=0;sensor_id_i< this->params_vector_.size(); sensor_id_i++){

                Eigen::Matrix4d &T_imu0_sensor_i = this->getParamsAt<ParameterType>(sensor_id_i)->T_imu0_sensor_;

                for(int sensor_id_j=0;sensor_id_j<(int)this->params_vector_.size(); sensor_id_j++){

                    Eigen::Matrix4d &T_imu0_sensor_j = this->getParamsAt<ParameterType>(sensor_id_j)->T_imu0_sensor_;
                    
                    Eigen::Matrix4d T_sensor_i_sensor_j = T_imu0_sensor_i.inverse() * T_imu0_sensor_j;

                    this->sensor_id_sensor_id_extrinsics_map_[{sensor_id_i, sensor_id_j}] = Sophus::SE3d::fitToSE3(T_sensor_i_sensor_j);

                }
            }
            
        }



        Eigen::Matrix<double, 4, 4> getExtrinsicsBetweenCamerasBySensorID(const unsigned int sensor_id_i, const unsigned int sensor_id_j);

        template<typename ParameterType>
        std::shared_ptr<ParameterType> getParamsAt(int idx) const{
            if(idx < 0 || idx >= static_cast<int>(params_vector_.size())){
                return nullptr;
            }

            return std::dynamic_pointer_cast<ParameterType>(params_vector_.at(idx));
        }

    public:
        std::shared_ptr<cv::FileStorage> file_storage_ = nullptr;
        std::string path_;
        std::vector<std::shared_ptr<Parameters>> params_vector_;

        std::map<std::pair<unsigned int, unsigned int>, Sophus::SE3d> sensor_id_sensor_id_extrinsics_map_;


};

} // namespace modules_vins