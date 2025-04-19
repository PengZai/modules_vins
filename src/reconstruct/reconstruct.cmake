


list(APPEND LIBRARY_SOURCES

    ${CMAKE_CURRENT_SOURCE_DIR}/src/reconstruct/reconstructor.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/reconstruct/sensor_depths/sensor_depth_reconstruction.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/reconstruct/two_view_depths/two_view_reconstruction.cc


)


if(USE_LIBTORCH)

    list(APPEND LIBRARY_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/reconstruct/depth_estimations/MiDas_reconstruction.cc
    )

endif()