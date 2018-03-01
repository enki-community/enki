set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)


add_library(enkiviewer
	Viewer.cpp
	EPuckModel.cpp
	objects/EPuckBody.cpp
	objects/EPuckRest.cpp
	objects/EPuckRing.cpp
	objects/EPuckWheelLeft.cpp
	objects/EPuckWheelRight.cpp
	MarxbotModel.cpp
	objects/MarxbotBase.cpp
	objects/MarxbotWheel.cpp
	Thymio2Model.cpp
	objects/Thymio2Body.cpp
	objects/Thymio2Wheel.cpp
	enki-viewer-textures.qrc

)

set_target_properties(enkiviewer PROPERTIES VERSION ${LIB_VERSION_STRING}
									SOVERSION ${LIB_VERSION_MAJOR})

target_include_directories(enkiviewer PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(enkiviewer LINK_PUBLIC enki Qt5::Widgets Qt5::OpenGL ${OPENGL_LIBRARIES})

set(ENKI_VIEWER_HDR
	Viewer.h
)
install(FILES ${ENKI_VIEWER_HDR}
	DESTINATION include/viewer/
)
install(TARGETS enkiviewer ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
								LIBRARY DESTINATION ${LIB_INSTALL_DIR})
