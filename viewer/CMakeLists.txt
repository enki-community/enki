# SDL
find_package(SDL)
if (SDL_FOUND)
	include_directories(${SDL_INCLUDE_DIR})
	set(EXTRA_LIBS ${SDL_LIBRARY} ${EXTRA_LIBS})
	if (APPLE)
		execute_process(COMMAND "sdl2-config" "--static-libs" OUTPUT_VARIABLE SDL2_EXTRAS)
		string(STRIP ${SDL2_EXTRAS} SDL2_EXTRAS)
		set(EXTRA_LIBS ${SDL2_EXTRAS} ${EXTRA_LIBS})
	endif (APPLE)
	add_definitions("-DUSE_SDL")
	message(STATUS "SDL found")
endif (SDL_FOUND)

set(QT_USE_QTOPENGL TRUE)
find_package(Qt4)
find_package(OpenGL)

if (QT4_FOUND AND OPENGL_FOUND)
	include(${QT_USE_FILE})
	add_definitions(${QT_DEFINITIONS})
	
	if (CMAKE_HOST_UNIX)
		add_definitions("-fPIC")
	endif (CMAKE_HOST_UNIX)
	
	set(viewer_lib_SRCS
		Viewer.cpp
		EPuckModel.cpp
		objects/EPuckBody.cpp
		objects/EPuckRest.cpp
		objects/EPuckRing.cpp
		objects/EPuckWheelLeft.cpp
		objects/EPuckWheelRight.cpp
		MarxbotModel.cpp
		objects/MarxbotBase.cpp
		objects/MarxbotWheel.cpp)
	set(viewer_test_SRCS Playground.cpp)
	
	qt4_automoc(${viewer_lib_SRCS})
	qt4_automoc(${viewer_test_SRCS})
	
	qt4_add_resources(viewer_lib_RCC_SRCS enki-viewer-textures.qrc)
	
	include_directories (${enki_SOURCE_DIR} ${OPENGL_INCLUDE_DIR} ${CMAKE_BINARY_DIR}/viewer)
	
	link_directories (${enki_BINARY_DIR})
	
	add_library (enkiviewer ${viewer_lib_SRCS} ${viewer_lib_RCC_SRCS})
	set_target_properties(enkiviewer PROPERTIES VERSION ${LIB_VERSION_STRING} 
                                        SOVERSION ${LIB_VERSION_MAJOR})
	target_link_libraries(enkiviewer enki ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${EXTRA_LIBS})

	add_executable(enkiplayground ${viewer_test_SRCS})
	
	target_link_libraries(enkiplayground enkiviewer enki ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${EXTRA_LIBS})

	set(ENKI_VIEWER_HDR
		Viewer.h
	)
	install(FILES ${ENKI_VIEWER_HDR}
		DESTINATION include/viewer/
	)
	install(TARGETS enkiviewer ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
                                   LIBRARY DESTINATION ${LIB_INSTALL_DIR})
endif (QT4_FOUND AND OPENGL_FOUND)
