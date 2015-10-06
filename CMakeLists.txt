cmake_minimum_required (VERSION 2.8.4)

set (CMAKE_BUILD_TYPE "Release" CACHE STRING "Debug/Release")
set (CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/generator" CACHE PATH "Install prefix")


project (generator)
set (PROJECT_VERSION "1.2")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# It is necessary to enable C++11 support for GCC or Clang compilers.
	add_definitions ("-std=c++11 -Wall -pedantic")
	# We have added '-static-libstdc++' linking flag
	# to workaround the bug with broken ABI of libstdc++ gcc libraries
	# (gcc-4.7, clang-3.3 using the libraries of gcc-4.8). It was resulting in 
	# invalid time measurement if the chrono extension of C++11 library was used.
	set (CMAKE_EXE_LINKER_FLAGS "-static-libstdc++" CACHE STRING "" FORCE)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	# Enable C++11 support for Intel compiler.
	if (WIN32)
		add_definitions ("/Qstd=c++11")
	endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	# Enable exception handling for Microsoft MSVC compiler.
	add_definitions ("/EHsc")
else ()
	message (FATAL_ERROR "Unsupported compiler '${CMAKE_CXX_COMPILER_ID}'!")
endif()

# Generate configuration file for the program (PROJECT_BINARY_DIR is the building directory).
configure_file (
	"conf/GeneratorConfig.h.in"
	"${PROJECT_BINARY_DIR}/GeneratorConfig.h"
	)

include_directories ("inc" "${PROJECT_BINARY_DIR}")
add_subdirectory ("src")

# Generating documentation, inspired from 'https://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/'.
find_package (Doxygen)
if (DOXYGEN_FOUND)
	configure_file ("conf/doxyfilehtml.in" "${PROJECT_BINARY_DIR}/doxyfilehtml" @ONLY)
	add_custom_target (doc
		${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/doxyfilehtml
		WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM
	)
	file (MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/doc")
endif ()

# Install miscellaneous files (default parameters, xml schema)...
install_files ("/share/${PROJECT_NAME}-${PROJECT_VERSION}/"
	FILES "default_configuration/project_configuration.cfg" "xmlschema/dataset.xsd")

if (IS_DIRECTORY "${PROJECT_BINARY_DIR}/doc/")
	install (DIRECTORY "${PROJECT_BINARY_DIR}/doc" DESTINATION "share")
endif ()
