cmake_minimum_required(VERSION 3.3)
project(modules_vins)


find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
find_package(glog 0.6.0 REQUIRED)


message(STATUS "OPENCV: " ${OpenCV_VERSION} " | BOOST: " ${Boost_VERSION} " | CERES: " ${Ceres_VERSION})


# We need c++14 for ROS2, thus just require it for everybody
# NOTE: To future self, hope this isn't an issue...
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Setting CMAKE_CXX_EXTENSIONS to OFF ensures strict compliance with the selected C++ standard.

option(ENABLE_CMAKE_DEBUG "Enable detailed debug messages" ON)

# Debug or Release
set(CMAKE_BUILD_TYPE "Debug")

# Enable compile optimizations
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Wall -Wuninitialized -fno-omit-frame-pointer")



# By default we build with ROS, but you can disable this and just build as a library
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)

if(ENABLE_ROS)
    find_package(catkin QUIET)  # ROS1
    find_package(ament_cmake QUIET) # ROS2
    if(catkin_FOUND)
        message(STATUS "ROS *1* version found, building ROS1.cmake")
        include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
    elseif(ament_cmake_FOUND)
        message(STATUS "ROS *2* version found, building ROS2.cmake")
        include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
    endif()

else()
    message(STATUS "Disabled building with ROS, building project as library in Library.cmake withou any ROS")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Library.cmake)
endif()

    






