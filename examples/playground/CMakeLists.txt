# SDL
find_package(SDL2)
if (SDL2_FOUND)
	message(STATUS "SDL2 found, enabling joystick support")
	include_directories(${SDL2_INCLUDE_DIR})
	set(EXTRA_LIBS ${SDL2_LIBRARY} ${EXTRA_LIBS})
	add_definitions("-DUSE_SDL")
endif (SDL2_FOUND)

if (QT4_FOUND AND OPENGL_FOUND)
	include(${QT_USE_FILE})
	add_definitions(${QT_DEFINITIONS})
	
	set(viewer_playground_SRCS Playground.cpp)
	qt4_automoc(${viewer_playground_SRCS})
	
	include_directories (${PROJECT_SOURCE_DIR} ${OPENGL_INCLUDE_DIR})
	
	add_executable(enkiplayground ${viewer_playground_SRCS})
	
	target_link_libraries(enkiplayground enkiviewer enki ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${EXTRA_LIBS})
endif (QT4_FOUND AND OPENGL_FOUND)
