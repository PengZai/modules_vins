



include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system
)



list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/vins_system.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/system_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/camera_config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system/imu_config.cc

)


