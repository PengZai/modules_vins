




# target_include_directories(run_serial_vins
#     PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/system
# )



list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/vins_system.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/state.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/system_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/camera_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/imu_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/visualizer_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/feature_and_tracker_config.cc

)


