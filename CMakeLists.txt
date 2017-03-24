cmake_minimum_required(VERSION 2.6)

project(Enki)

# additional CMake modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

set(LIB_INSTALL_DIR lib CACHE FILEPATH "Where to install libraries")
set(LIB_VERSION_MAJOR 2) # Must be bumped for incompatible ABI changes
set(LIB_VERSION_MINOR 0)
set(LIB_VERSION_PATCH 0)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

# enable C++11 support
if (CMAKE_VERSION VERSION_LESS "3.1")
        if (MSVC)
                message(FATAL_ERROR, "CMake version 3.1 or later is required to compiler Aseba with Microsoft Visual C++")
        endif ()
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
                set (CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
        else ()
                set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
        endif ()
else ()
        set (CMAKE_CXX_STANDARD 11)
endif ()

# Load settings for external projects that provide a CMake config file.
find_package(dashel REQUIRED)
include_directories(${dashel_INCLUDE_DIRS})

find_package(Threads)

add_subdirectory(enki)
add_subdirectory(examples)
add_subdirectory(viewer)
add_subdirectory(python)

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

# testing
enable_testing()
add_subdirectory(tests)
