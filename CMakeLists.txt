cmake_minimum_required(VERSION 2.8.11)

project(Enki)

# additional CMake modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

enable_testing()

# check for Qt
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)

# check for SDL2
find_package(SDL2)

# library version
set(LIB_INSTALL_DIR lib CACHE FILEPATH "Where to install libraries")
set(LIB_VERSION_MAJOR 2) # Must be bumped for incompatible ABI changes
set(LIB_VERSION_MINOR 0)
set(LIB_VERSION_PATCH 0)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

add_subdirectory(enki)
add_subdirectory(viewer)
add_subdirectory(python)
add_subdirectory(tests)
add_subdirectory(examples)

# Documentation
find_package(Doxygen)
option(BUILD_DOCUMENTATION
	"Create and install the HTML based API documentation (requires Doxygen)"
	${DOXYGEN_FOUND})

if (BUILD_DOCUMENTATION)
	if (NOT DOXYGEN_FOUND)
		message(FATAL_ERROR "Doxygen is needed to build the documentation.")
	endif()

	set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
	set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

	configure_file(${doxyfile_in} ${doxyfile} @ONLY)

	add_custom_target(doc
		COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen"
		VERBATIM)

	install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc OPTIONAL)
endif()

# generate configure file
configure_file(enkiConfig.cmake.in "${PROJECT_BINARY_DIR}/enkiConfig.cmake" @ONLY)
if (WIN32 AND NOT CYGWIN)
	set(INSTALL_CMAKE_DIR CMake)
else()
	if (APPLE)
		set(INSTALL_CMAKE_DIR Frameworks/enki.framework/Resources/CMake/)
	else()
		set(INSTALL_CMAKE_DIR share/enki/CMake)
	endif()
endif()
install(FILES "${PROJECT_BINARY_DIR}/enkiConfig.cmake" DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
