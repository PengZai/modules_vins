



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

    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/detector.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/features/feature_points.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/tracking/tracker.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/tracking/tracking_descriptor.cc

    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/reconstructor.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/sensor_depths/sensor_depth_reconstruction.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/two_view_depths/two_view_reconstruction.cc

    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/pose_estimate/pose_estimator.cc


)


if(USE_LIBTORCH)

    
# target_include_directories(run_serial_vins
#         PRIVATE
#         # ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/object_detections/depth_estimations
#         # ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/object_detections
#         # ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/semantic_segmentations
#     )
    list(APPEND LIBRARY_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/depth_estimations/MiDas_reconstruction.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/object_detections/yolo_detector.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/semantic_segmentations/yolo_segmentor.cc
    )

endif()


# ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/features/descriptors.cc
# ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/tracking/tracking_KLT.cc


