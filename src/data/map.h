#pragma once

#include <iostream>
#include <memory>
#include <map>

#include "../log/logging.h"
#include "camera.h"
#include "point.h"


namespace modules_vins{

class CameraFrame;


class Map{

    public:
    Map();

    void insertMapPoint(const std::shared_ptr<MapPoint> &mappoint);
    bool isExistedMapPoint(const std::shared_ptr<MapPoint> &mappoint);
    void update(const CameraFrame &camera_frame);
    const std::map<unsigned int, std::shared_ptr<MapPoint>>& getMapPoints() const;


    protected:

    std::map<unsigned int, std::shared_ptr<MapPoint>> mappoints_;
};


} //modules_vins

