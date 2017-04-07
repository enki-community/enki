include_directories (${PROJECT_SOURCE_DIR})
link_directories (${PROJECT_BINARY_DIR})

add_executable(enkiMinimal enkiMinimal.cpp)
target_link_libraries(enkiMinimal enki)

