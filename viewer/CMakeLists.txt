find_package(Qt4 REQUIRED)

if (QT4_FOUND)
	include(${QT_USE_FILE})

	set(viewer_lib_SRCS Viewer.cpp objects/EPuckBody.cpp objects/EPuckRest.cpp objects/EPuckRing.cpp objects/EPuckWheelLeft.cpp objects/EPuckWheelRight.cpp)
	set(viewer_test_SRCS Studio.cpp)

	qt4_automoc(${viewer_lib_SRCS})
	qt4_automoc(${viewer_test_SRCS})
	
	qt4_add_resources(viewer_lib_RCC_SRCS textures.qrc)

	include_directories (${enki_SOURCE_DIR} ${QT_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR})

	link_directories (${enki_BINARY_DIR})

	add_library (enkiviewer ${viewer_lib_SRCS} ${viewer_lib_RCC_SRCS})
	add_executable(enkistudio ${viewer_test_SRCS})

	target_link_libraries(enkistudio ${QT_QTOPENGL_LIBRARIES} ${QT_LIBRARIES} enkiviewer enki)
endif (QT4_FOUND)
