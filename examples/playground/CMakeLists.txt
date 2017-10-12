if (SDL2_FOUND)
	message(STATUS "SDL2 found, enabling joystick support")
	include_directories(${SDL2_INCLUDE_DIR})
	set(EXTRA_LIBS ${SDL2_LIBRARY} ${EXTRA_LIBS})
	add_definitions("-DUSE_SDL")
endif ()

if (Qt5Widgets_FOUND AND Qt5OpenGL_FOUND)
	set(CMAKE_AUTOMOC ON)
	set(viewer_playground_SRCS Playground.cpp)
	
	include_directories(${PROJECT_SOURCE_DIR})
	
	add_executable(enkiplayground ${viewer_playground_SRCS})
	
	target_link_libraries(enkiplayground enkiviewer enki ${OPENGL_gl_LIBRARY} ${EXTRA_LIBS} Qt5::OpenGL Qt5::Widgets)
endif ()
