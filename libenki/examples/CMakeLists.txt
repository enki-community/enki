include_directories (${enki_SOURCE_DIR})
link_directories (${enki_BINARY_DIR})

add_executable(enkiTest enkitest.cpp)

target_link_libraries(enkiTest enki)