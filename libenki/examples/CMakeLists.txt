include_directories (${enki_SOURCE_DIR})
link_directories (${enki_BINARY_DIR})

add_executable(enkiTest enkiTest.cpp)
target_link_libraries(enkiTest enki)

if (ASEBA_FOUND)
	include_directories (${ASEBA_INCLUDE_DIR})
	link_directories (${enkiaseba_BINARY_DIR})

	add_executable(asebaTest asebaTest.cpp)
	target_link_libraries(asebaTest enkiaseba enki  ${ASEBA_VM_LIBRARY})
endif (ASEBA_FOUND)
