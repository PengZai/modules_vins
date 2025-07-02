




# target_include_directories(run_serial_vins
#     PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/data
# )



list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/dataloader.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/camera.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/point.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/map.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/frame.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/preprocess.cc

)


