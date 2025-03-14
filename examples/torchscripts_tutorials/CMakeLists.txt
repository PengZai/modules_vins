cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(custom_ops)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_PREFIX_PATH /root/libtorch)
find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
)

add_executable(example-app 
                example-app.cpp
                YoloDetect.cpp
                )


add_executable(example-seg 
                example-seg.cpp
                )

target_link_libraries(example-app 
            "${TORCH_LIBRARIES}"
            "${OpenCV_LIBS}"
            )
set_property(TARGET example-app PROPERTY CXX_STANDARD 17)

target_link_libraries(example-seg 
        "${TORCH_LIBRARIES}"
        "${OpenCV_LIBS}"
        )
set_property(TARGET example-seg PROPERTY CXX_STANDARD 17)