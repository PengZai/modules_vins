list(APPEND LIBRARY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/log/logging.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/log/evo_record.cc

)


# target_include_directories(run_serial_vins
#     PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/log
# )