



# target_include_directories(run_serial_vins
#     PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/features
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/tracking
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/sensor_depths
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/two_view_depths
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/pose_estimate
# )



list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/visual_frontend.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/descriptor_match_frontend.cc



)




