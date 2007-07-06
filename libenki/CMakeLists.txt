project(enki)

include(FindAseba.cmake)

add_subdirectory(enki)
if (ASEBA_FOUND)
	add_subdirectory(aseba)
endif (ASEBA_FOUND)
add_subdirectory(examples)
