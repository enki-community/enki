find_path(ASEBA_INCLUDE_DIR vm/vm.h ${withAseba})
find_library(ASEBA_VM_LIBRARY asebavm ${withAseba}/vm/build-local)

if (ASEBA_INCLUDE_DIR AND ASEBA_VM_LIBRARY)
	set (ASEBA_FOUND true)
endif (ASEBA_INCLUDE_DIR AND ASEBA_VM_LIBRARY)

if (ASEBA_FOUND)
	message(STATUS "Aseba found at ${withAseba}")
else (ASEBA_FOUND)
	message(STATUS "Could not find aseba")
endif (ASEBA_FOUND)
