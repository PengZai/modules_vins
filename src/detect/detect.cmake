


list(APPEND LIBRARY_SOURCES

    ${CMAKE_CURRENT_SOURCE_DIR}/src/detect/detector.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/detect/features/feature_points.cc
)


if(USE_LIBTORCH)

    

    list(APPEND LIBRARY_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/detect/object_detections/yolo_detector.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/src/detect/semantic_segmentations/yolo_segmentor.cc
    )

endif()


# ${CMAKE_CURRENT_SOURCE_DIR}/src/detect/features/descriptors.cc
# ${CMAKE_CURRENT_SOURCE_DIR}/src/tracking/tracking_KLT.cc
