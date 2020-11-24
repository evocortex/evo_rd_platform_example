include(FindPackageHandleStandardArgs)

find_package(evo-mbed-tools NO_MODULE)
if(DEFINED evo-mbed-tools_FOUND)
	find_package_handle_standard_args(evo-mbed-tools CONFIG_MODE)
	return()
endif()

find_path(EVO_MBED_TOOLS_INCLUDE NAMES evo_mbed
	PATHS /usr/include/evo_mbed
	)

if(DEFINED EVO_MBED_TOOLS_INCLUDE)
	file(READ ${EVO_MBED_TOOLS_INCLUDE}/Version.h version_file)

	string(REGEX MATCH "EVO_MBED_TOOLS_VER\s+\"(\d\.\d\.\d)\"" _ ${version_file})
	set(evo-mbed-tools_VERSION ${CMAKE_MATCH_1} CACHE INTERNAL)
	set(evo-mbed-tools_INCLUDE_DIR ${EVO_MBED_TOOLS_INCLUDE} CACHE INTERNAL)
endif()

find_library(LIBEVO_MBED_TOOLS NAMES
	evo-mbed-tools
	libevo-mbed-tools
	libevo-mbed-tools.so
	libevo-mbed-tools.so.1
	PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/libevo-mbed-tools.so.1
	)

if(DEFINED LIBEVO_MBED_TOOLS AND NOT TARGET evo-mbed-tools::evo-mbed-tools)
	add_library(evo-mbed-tools::evo-mbed-tools UNKNOWN IMPORTED)
	set_target_properties(evo-mbed-tools::evo-mbed-tools PROPERTIES
		IMPORTED_LOCATION ${LIBEVO_MBED_TOOLS}
		)

	set(evo-mbed-tools_LIBRARY ${LIBEVO_MBED_TOOLS} CACHE INTERNAL)
endif()

find_package_handle_standard_args(evo-mbed-tools
	REQUIRED_VARS evo-mbed-tools_INCLUDE_DIR evo-mbed-tools_LIBRARY
	VERSION_VAR evo-mbed-tools_VERSION
	)

mark_as_advanced(evo-mbed-tools_INCLUDE_DIR
	evo-mbed-tools_LIBRARY
	evo-mbed-tools_VERSION
	)

