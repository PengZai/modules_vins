#pragma once

#include <iostream>
#include <memory>
#include <map>

#include "../system/system_config.h"
#include "../log/logging.h"
#include "camera.h"
#include "point.h"


namespace modules_vins{


class Map{

    public:
    Map(const std::shared_ptr<modules_vins::SystemConfig> &sys_config);

    void insertMapPoint(const std::shared_ptr<MapPoint> &mappoint);
    bool isExistedMapPoint(const std::shared_ptr<MapPoint> &mappoint);
    void update(const std::shared_ptr<CameraFrame> &camera_frame);
    const std::map<unsigned int, std::shared_ptr<MapPoint>>& getMapPoints() const;
    void maintainSize();


    public:
    std::shared_ptr<modules_vins::SystemConfig> sys_config_;

    protected:

    std::map<unsigned int, std::shared_ptr<MapPoint>> mappoints_;
};


} //modules_vins

