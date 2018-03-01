add_library(enki
	Geometry.cpp
	Types.cpp
	PhysicalEngine.cpp
	BluetoothBase.cpp
	interactions/IRSensor.cpp
	interactions/GroundSensor.cpp
	interactions/CircularCam.cpp
	interactions/Bluetooth.cpp
	interactions/ActiveSoundSource.cpp
	interactions/Microphone.cpp
	robots/DifferentialWheeled.cpp
	robots/khepera/Khepera.cpp
	robots/e-puck/EPuck.cpp
	robots/s-bot/Sbot.cpp
	robots/s-bot/SbotObject.cpp
	robots/marxbot/Marxbot.cpp
	robots/thymio2/Thymio2.cpp
)

target_include_directories (enki PUBLIC ${PROJECT_SOURCE_DIR})

set_target_properties(enki PROPERTIES VERSION ${LIB_VERSION_STRING}
										SOVERSION ${LIB_VERSION_MAJOR}
										POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY . 
	DESTINATION include/enki/
	FILES_MATCHING PATTERN "*.h"
)
install(TARGETS enki LIBRARY DESTINATION ${LIB_INSTALL_DIR}
                     ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
