cmake_minimum_required (VERSION 2.6)
project(reset-arduino)

set(LIBS_DIR libs)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/${LIBS_DIR})
add_subdirectory(${CMAKE_SOURCE_DIR}/${LIBS_DIR})

# add executable
set(SOURCES main.cpp ${HEADER_FILES})
add_executable(reset-arduino main.cpp)
target_link_libraries(reset-arduino Arduino)

# set platform compiler flags
# modified from http://www.openguru.com/2009/04/cmake-detecting-platformoperating.html
if(${WIN32})
	set_property(
		TARGET reset-arduino Arduino
		PROPERTY COMPILE_DEFINITIONS TARGET_WIN32
	)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	set_property(
		TARGET reset-arduino Arduino
		PROPERTY COMPILE_DEFINITIONS TARGET_OSX
	)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	set_property(
		TARGET reset-arduino Arduino
		PROPERTY COMPILE_DEFINITIONS TARGET_LINUX
	)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")