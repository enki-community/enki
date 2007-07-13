include_directories (${enki_SOURCE_DIR})
link_directories (${enki_BINARY_DIR})

add_executable(enkiTest enkiTest.cpp)
target_link_libraries(enkiTest enki)

