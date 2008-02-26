project(enki)

add_subdirectory(enki)

add_subdirectory(examples)

find_package(Qt4)
if (QT4_FOUND)
	add_subdirectory(viewer)
endif (QT4_FOUND)

