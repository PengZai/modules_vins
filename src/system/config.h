#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <Eigen/Dense>


#include "../log/logging.h"




namespace modules_vins
{   


class Config
{
    public:

        virtual ~Config() = default;

        virtual void loadConfigFromPath(const std::string &config_path) = 0;

        
    public:
        std::shared_ptr<cv::FileStorage> file_storage_ = nullptr;
        std::string path_;

};


class Parameters{
 
    public:

        virtual ~Parameters() = default;
        virtual void loadFromNode(const std::shared_ptr<cv::FileNode> &node) = 0;


        std::shared_ptr<cv::FileNode> node_;

        template <typename T>
        void parse(const std::string &parameter_name, T &parsed_value){
            (*this->node_)[parameter_name] >> parsed_value;
            VLOG(VERBOSE) << parameter_name << " : " << parsed_value;
        }

        template <typename T>
        void parse(const std::string &parameter_name, std::vector<T> &parsed_values){
            std::stringstream log_stream;

            (*this->node_)[parameter_name] >> parsed_values;

            log_stream << parameter_name << " : [ ";

            for (size_t i = 0; i < parsed_values.size(); ++i) {
                log_stream << parsed_values[i] << ", ";
            }
            
            VLOG(VERBOSE) << log_stream.str() << "]";
 
        }

        void parse(const std::string &parameter_name, Eigen::VectorXd &parsed_values){
            std::stringstream log_stream;
            
            cv::FileNode node = (*this->node_)[parameter_name];
            log_stream << parameter_name << " : [ ";

            for (size_t i = 0; i < node.size(); ++i) {
                parsed_values(i) = static_cast<double>(node[i]);
                log_stream << parsed_values(i) << ", ";
                
            }
            
            VLOG(VERBOSE) << log_stream.str() << "]";
 
        }

        void parse(const std::string &parameter_name, Eigen::Matrix4d parsed_values){
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

            VLOG(VERBOSE) << log_stream.str() << "]";


        }

};

} // namespace modules_vins