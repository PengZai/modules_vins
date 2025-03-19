



include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data
)



list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/camera.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/point.cc
)


