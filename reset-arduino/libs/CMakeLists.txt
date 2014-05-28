set(HEADER_FILES Arduino.h)
add_library(Arduino STATIC Arduino.cpp ${HEADER_FILES})

if(${WIN32})
	target_link_libraries(Arduino setupapi)
endif()