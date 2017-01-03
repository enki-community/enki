# Testing for Enki

include_directories(
	${enki_SOURCE_DIR}
)
link_directories(
	${enki_BINARY_DIR}
)

add_executable(enkiTestBDD
        enkiTestBDD.cpp
)
target_link_libraries(enkiTestBDD
	enki
)

# the following tests should succeed
add_test(enki-test enkiTestBDD)
