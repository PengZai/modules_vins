cmake_minimum_required(VERSION 3.3)


set(LIB_PROJECT_NAME "${PROJECT_NAME}")





# file(GLOB_RECURSE HEADERS_LIBRARY "src/*.h")

# message(STATUS "HEADERS_LIBRARY: " ${HEADERS_LIBRARY})



# ==============================ros================================= 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  rospy
  std_msgs
  sensor_msgs 
  image_transport 
  cv_bridge
)

catkin_package(
    CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    sensor_msgs 
    image_transport 
    cv_bridge
    LIBRARIES ${LIB_PROJECT_NAME} 

)
# ==============================ros=================================


list(APPEND LIBRARY_THIRDPARTY
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    glog::glog
    ${catkin_LIBRARIES}
)



list(APPEND DIRS_HEADER_THIRDPARTY
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

list(APPEND LIBRARY_SOURCES

)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${DIRS_HEADER_THIRDPARTY}
)


# whenever creating a new .cc file, add it in .cmake in that subdirectory
include(${CMAKE_CURRENT_SOURCE_DIR}/src/system/system.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/log/log.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/data/data.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/frontend.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/visualization.cmake)





add_library(${LIB_PROJECT_NAME}
    SHARED
    ${LIBRARY_SOURCES}
)
target_link_libraries(${LIB_PROJECT_NAME}
    PUBLIC
    ${LIBRARY_THIRDPARTY}
)



add_executable(run_serial_vins
    src/run_serial_vins.cc
)

target_link_libraries(run_serial_vins
    PUBLIC
    ${LIB_PROJECT_NAME}
)


if(ENABLE_CMAKE_DEBUG)
    get_property(INCLUDE_DIRS DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
    message(STATUS "Include Directories: ${INCLUDE_DIRS}")

    get_property(TARGETS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY BUILDSYSTEM_TARGETS)

    foreach(target IN LISTS TARGETS)
        get_target_property(SOURCE_FILES ${target} SOURCES)
        if(SOURCE_FILES)
            message(STATUS "Target: ${target}")
            foreach(file IN LISTS SOURCE_FILES)
                message(STATUS "  Linked Source: ${file}")
            endforeach()
        endif()
    endforeach()
else()

endif()


# install(TARGETS run_serial_slam
#             ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#             LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#             RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )


