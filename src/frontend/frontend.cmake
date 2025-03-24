



include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/features
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/tracking
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/pose_estimate
)



list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/visual_frontend.cc

    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/detector.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/features/feature_points.cc

    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/tracking/tracker.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/tracking/tracking_descriptor.cc

    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/reconstructor.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/two_view_reconstruction.cc

    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/pose_estimate/pose_estimator.cc


)


# ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/detect/features/descriptors.cc
# ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/tracking/tracking_KLT.cc


