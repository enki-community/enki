# Testing for Enki

include_directories(
	${enki_SOURCE_DIR}
)
link_directories(
	${enki_BINARY_DIR}
)

# Contain Catch testing framework main function.  Using CATCH_CONFIG_MAIN macro
# only this library reduce compilation time.
add_library(catch STATIC
	catch.cpp
	catch.hpp
)

# BDD test example.
add_executable(enkiTestBDD
        enkiTestBDD.cpp
)
target_link_libraries(enkiTestBDD enki catch)

add_test(enki-test enkiTestBDD "An Enki world may contain a robot")

# Test the World class.
add_executable(enkiTestWorld enkiTestWorld.cpp)
target_link_libraries(enkiTestWorld enki catch)

add_test(enki_test_empty_world enkiTestWorld "An empty Enki world")
add_test(enki_test_thymio enkiTestWorld "Thymio2")
add_test(enki_test_EPuck enkiTestWorld "EPuck")
add_test(enki_test_Marxbot enkiTestWorld "Marxbot")
add_test(enki_test_Khepera enkiTestWorld "Khepera")
add_test(enki_test_Sbot enkiTestWorld "S-Bot")

add_executable(randomWorld randomWorld.cpp Randomizer.cpp)
target_link_libraries(randomWorld catch enki)

add_test(random_world randomWorld)
