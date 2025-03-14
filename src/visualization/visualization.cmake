



include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualization
)



list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/opencv_visualizer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/ros1_visualizer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/visualizer.cc
)


