cmake_minimum_required(VERSION 2.8.10)

project(enki)

# check the compiler version as we need full C++11 support
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# using Clang
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.4")
		message(FATAL_ERROR, "Your clang compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while version 3.4 or later is required")
	endif ()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
	# using AppleClang
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.1")
		message(FATAL_ERROR "Your XCode environment has version ${CMAKE_CXX_COMPILER_VERSION}, while version 5.1 or later is required")
	endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	# using GCC
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.9")
		message(FATAL_ERROR, "Your g++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while version 4.9 or later is required")
	endif ()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	# using MSVC
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.0.23506")
		message(FATAL_ERROR "Your Microsoft Visual C++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while version MSVC 2015 Update 1+ is required")
	endif()
endif ()

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

set(LIB_INSTALL_DIR lib CACHE FILEPATH "Where to install libraries")
set(LIB_VERSION_MAJOR 2) # Must be bumped for incompatible ABI changes
set(LIB_VERSION_MINOR 0)
set(LIB_VERSION_PATCH 0)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})
add_subdirectory(enki)
add_subdirectory(examples)
add_subdirectory(viewer)
add_subdirectory(python)
