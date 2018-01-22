find_package(Qt5 5.6 COMPONENTS Core Gui Widgets OpenGL)
find_package(OpenGL REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(${PROJECT_SOURCE_DIR})

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

)

set_target_properties(enkiviewer PROPERTIES VERSION ${LIB_VERSION_STRING}
									SOVERSION ${LIB_VERSION_MAJOR})
target_link_libraries(enkiviewer enki)
qt5_use_modules(enkiviewer Widgets OpenGL)

set(ENKI_VIEWER_HDR
	Viewer.h
)
install(FILES ${ENKI_VIEWER_HDR}
	DESTINATION include/viewer/
)
install(TARGETS enkiviewer ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
								LIBRARY DESTINATION ${LIB_INSTALL_DIR})
