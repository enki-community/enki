# SDL
find_package(SDL)
if (SDL_FOUND)
	include_directories(${SDL_INCLUDE_DIR})
	set(EXTRA_LIBS ${SDL_LIBRARY} ${EXTRA_LIBS})
	add_definitions("-DUSE_SDL")
endif (SDL_FOUND)

find_package(Qt4)

if (QT4_FOUND)

	find_package(OpenGL REQUIRED)
	
	include(${QT_USE_FILE})
	
	set(viewer_lib_SRCS Viewer.cpp objects/EPuckBody.cpp objects/EPuckRest.cpp objects/EPuckRing.cpp objects/EPuckWheelLeft.cpp objects/EPuckWheelRight.cpp)
	set(viewer_test_SRCS Playground.cpp)
	
	qt4_automoc(${viewer_lib_SRCS})
	qt4_automoc(${viewer_test_SRCS})
	
	qt4_add_resources(viewer_lib_RCC_SRCS enki-viewer-textures.qrc)
	
	include_directories (${enki_SOURCE_DIR} ${QT_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${CMAKE_BINARY_DIR}/viewer)
	
	link_directories (${enki_BINARY_DIR})
	
	add_library (enkiviewer ${viewer_lib_SRCS} ${viewer_lib_RCC_SRCS})
	add_executable(enkiplayground ${viewer_test_SRCS})
	
	target_link_libraries(enkiplayground enkiviewer enki ${QT_QTOPENGL_LIBRARIES} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${EXTRA_LIBS})

endif (QT4_FOUND)
