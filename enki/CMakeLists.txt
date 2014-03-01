include_directories (${enki_SOURCE_DIR})

if (CMAKE_HOST_UNIX)
	add_definitions("-fPIC")
endif (CMAKE_HOST_UNIX)

add_library(enki
	Geometry.cpp
	Types.cpp
	PhysicalEngine.cpp
	BluetoothBase.cpp
	interactions/IRSensor.cpp
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
)

set_target_properties(enki PROPERTIES VERSION ${LIB_VERSION_STRING} 
                                        SOVERSION ${LIB_VERSION_MAJOR})

install(DIRECTORY . 
	DESTINATION include/enki/
	FILES_MATCHING PATTERN "*.h"
)
install(TARGETS enki LIBRARY DESTINATION ${LIB_INSTALL_DIR}
                     ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
