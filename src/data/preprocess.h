#pragma once

#include "frame.h"


namespace modules_vins{

class DataPreprocesor{

    public:

    void pipeline(const std::shared_ptr<Frame> &frame);


};



} //modules_vins