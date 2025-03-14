#pragma once

#include <Eigen/Dense>
#include <memory>
#include "config.h"


namespace modules_vins
{
    


class ImuConfig : public Config 
{
    public:
        void loadConfigFromPath(const std::string &config_path) override;

    public:
        Eigen::Matrix2d T_i_b_;
        double accelerometer_noise_density_; // [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
        double accelerometer_random_walk_;  // [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )
        double gyroscope_noise_density_;  // [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
        double gyroscope_random_walk_;    // [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
        std::string rostopic_;
        double time_offset_;
        double update_rate_;
        std::string model_;


};


} // namespace ddep_vins
