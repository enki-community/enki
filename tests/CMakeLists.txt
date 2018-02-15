add_executable(testGeometry testGeometry.cpp)
target_link_libraries(testGeometry enki)

# the following tests should succeed
add_test(NAME geometry COMMAND testGeometry)
