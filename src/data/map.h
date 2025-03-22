#pragma once
#include <iostream>
#include <memory>
#include <map>


#include "point.h"


namespace modules_vins{

class Map{

    Map();

    protected:

    std::map<unsigned int, std::shared_ptr<MapPoint>> mappoints_;

};


} //modules_vins

