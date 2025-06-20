
# target_include_directories(run_serial_vins
#     PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/init
# )


list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/init/initializer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/init/descriptor_match_initializer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/init/KLT_initializer.cc

)

