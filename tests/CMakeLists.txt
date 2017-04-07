add_executable(test-geometry test-geometry.cpp)
target_link_libraries(test-geometry enki)

# the following tests should succeed
add_test(geometry ${EXECUTABLE_OUTPUT_PATH}/test-geometry)
