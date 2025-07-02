#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <Eigen/Dense>


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
        void calculateExtrinsicsAndProjectionMatrixBetweenSensors();
        Eigen::Matrix<double, 4, 4> getExtrinsicsBetweenCamerasBySensorID(const unsigned int sensor_id_i, const unsigned int sensor_id_j);

    public:
        std::shared_ptr<cv::FileStorage> file_storage_ = nullptr;
        std::string path_;

        std::vector<std::shared_ptr<Parameters>> params_vector_;
        std::map<std::pair<unsigned int, unsigned int>, Eigen::Matrix4d> sensor_id_sensor_id_extrinsics_map_;


};

} // namespace modules_vins