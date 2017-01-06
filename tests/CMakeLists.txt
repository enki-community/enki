# Testing for Enki

include_directories(
	${enki_SOURCE_DIR}
)
link_directories(
	${enki_BINARY_DIR}
)

# BDD test example.
add_executable(enkiTestBDD
        enkiTestBDD.cpp
)
target_link_libraries(enkiTestBDD
	enki
)

add_test(enki-test enkiTestBDD "An Enki world may contain a robot")

# Test the World class.
add_executable(enkiTestWorld enkiTestWorld.cpp)
target_link_libraries(enkiTestWorld enki)

add_test(enki_test_empty_world enkiTestWorld "An empty Enki world")
