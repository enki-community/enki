include_directories (${enki_SOURCE_DIR})

add_library (enki
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
	robots/alice/Alice.cpp
	robots/alice/AliceCommunication.cpp
	robots/alice/AliceCam.cpp
	robots/s-bot/Sbot.cpp
	robots/s-bot/SbotObject.cpp
	robots/marxbot/Marxbot.cpp
)

install(DIRECTORY . 
	DESTINATION include/enki/
	FILES_MATCHING PATTERN "*.h"
)
install(TARGETS enki ARCHIVE DESTINATION lib)