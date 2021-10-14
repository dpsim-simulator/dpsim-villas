message("Configuring dpsim-villas...")

set(DPSIM_VILLAS_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/src/InterfaceSampleBased.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/InterfaceShmem.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/InterfaceVillas.cpp
    PARENT_SCOPE
)

set(DPSIM_VILLAS_LIBRARIES ${VILLASNODE_LIBRARIES} pthread PARENT_SCOPE)
set(DPSIM_VILLAS_INCLUDE_DIRS ${VILLASNODE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)

add_subdirectory(examples)