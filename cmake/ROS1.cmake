cmake_minimum_required(VERSION 3.3)


set(LIB_PROJECT_NAME "${PROJECT_NAME}")




# ==============================ros================================= 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  rospy
  std_msgs
  sensor_msgs 
  geometry_msgs
  nav_msgs
  image_transport 
  cv_bridge
  tf
)

catkin_package(
    CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    sensor_msgs 
    geometry_msgs
    nav_msgs
    image_transport 
    cv_bridge
    tf
)
# ==============================ros=================================



list(APPEND LIBRARY_THIRDPARTY
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    Sophus::Sophus
    ${GLOG_LIBRARIES}
    gflags
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARY_DIRS}
    ${CERES_LIBRARIES}
    ${catkin_LIBRARIES}
)


if(USE_LIBTORCH)
    list(APPEND LIBRARY_THIRDPARTY
        ${TORCH_LIBRARIES}
    )
endif()



list(APPEND DIRS_HEADER_THIRDPARTY
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

list(APPEND LIBRARY_SOURCES

)

# seems ros1 packages only are supported include_directories 
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)




add_definitions(${PCL_DEFINITIONS})


# whenever creating a new .cc file, add it in .cmake in that subdirectory
include(${CMAKE_CURRENT_SOURCE_DIR}/src/system/system.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/log/log.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/init/init.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/data/data.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/frontend.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/backend/backend.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/visualization.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/utils/utils.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/detect/detect.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/tracking/tracking.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/pose_estimate/pose_estimate.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/reconstruct/reconstruct.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/optimization/optimization.cmake)




add_library(${LIB_PROJECT_NAME}
    SHARED
    ${LIBRARY_SOURCES}
)
target_link_libraries(${LIB_PROJECT_NAME}
    PUBLIC
    ${LIBRARY_THIRDPARTY}
)


add_executable(run_serial_vins_KLT
    src/run_serial_vins_KLT.cc
)

target_include_directories(run_serial_vins_KLT
    PRIVATE
    ${DIRS_HEADER_THIRDPARTY}
)

target_link_libraries(run_serial_vins_KLT
    PUBLIC
    ${LIB_PROJECT_NAME}
)



add_executable(run_serial_vins_descriptor_match
    src/run_serial_vins_descriptor_match.cc
)

target_include_directories(run_serial_vins_descriptor_match
    PRIVATE
    ${DIRS_HEADER_THIRDPARTY}
)

target_link_libraries(run_serial_vins_descriptor_match
    PUBLIC
    ${LIB_PROJECT_NAME}
)



# if(ENABLE_CMAKE_DEBUG)
#     get_property(INCLUDE_DIRS DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
#     message(STATUS "Include Directories: ${INCLUDE_DIRS}")

#     get_property(TARGETS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY BUILDSYSTEM_TARGETS)

#     foreach(target IN LISTS TARGETS)
#         get_target_property(SOURCE_FILES ${target} SOURCES)
#         if(SOURCE_FILES)
#             message(STATUS "Target: ${target}")
#             foreach(file IN LISTS SOURCE_FILES)
#                 message(STATUS "  Linked Source: ${file}")
#             endforeach()
#         endif()
#     endforeach()
# else()

# endif()


# install(TARGETS run_serial_slam
#             ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#             LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#             RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )


