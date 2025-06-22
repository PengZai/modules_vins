

add_executable(test_orb_tracking
    ${CMAKE_CURRENT_SOURCE_DIR}/src/test_examples/test_orb_tracking.cc
)

target_include_directories(test_orb_tracking
    PRIVATE
    ${DIRS_HEADER_THIRDPARTY}
)

target_link_libraries(test_orb_tracking
    PUBLIC
    ${LIB_PROJECT_NAME}
)