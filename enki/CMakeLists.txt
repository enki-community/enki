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

set(ENKI_HDR
	Random.h
	interactions/ActiveSoundSource.h
	interactions/Bluetooth.h
	interactions/Microphone.h
	interactions/CircularCam.h
	interactions/IRSensor.h
	Interaction.h
	robots/e-puck/EPuck.h
	robots/khepera/Khepera.h
	robots/alice/Alice.h
	robots/alice/AliceCommunication.h
	robots/alice/AliceCam.h
	robots/s-bot/Sbot.h
	robots/s-bot/SbotObject.h
	robots/DifferentialWheeled.h
	robots/marxbot/Marxbot.h
	PhysicalEngine.h
	Types.h
	Geometry.h
	BluetoothBase.h
)
install(FILES ${ENKI_HDR}
	DESTINATION include/enki/
)
install(TARGETS enki ARCHIVE DESTINATION lib)