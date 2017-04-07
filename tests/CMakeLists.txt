add_executable(testGeometry testGeometry.cpp)
target_link_libraries(testGeometry enki)

# the following tests should succeed
add_test(geometry ${EXECUTABLE_OUTPUT_PATH}/testGeometry)
