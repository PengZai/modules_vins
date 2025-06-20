#include "preprocess.h"


namespace modules_vins
{



void DataPreprocesor::pipeline(const std::shared_ptr<CameraFrame> &camera_frame){


    for(int i=0;i<camera_frame->image_vector_.size();i++){


        const std::shared_ptr<Image> &img_i = camera_frame->image_vector_.at(i);

        if(img_i->color_data_.channels() == 3){
        cv::cvtColor(img_i->color_data_, img_i->gray_data_, cv::COLOR_BGR2GRAY);

        }
        else if(img_i->color_data_.channels() == 1){
            img_i->gray_data_ = img_i->color_data_;

        }
        else{
            LOG(INFO) << "img_i is neither the bgr image nor gray image";
            std::exit(EXIT_FAILURE);
        }

    }

    

    
}


    
    
} // namespace modules_vins
