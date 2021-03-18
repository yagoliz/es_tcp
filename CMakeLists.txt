cmake_minimum_required(VERSION 3.17)
project(es_tcp)

set(CMAKE_CXX_STANDARD 14)

# Look for our beloved libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

find_package( RTLSDR REQUIRED )
include_directories("${RTLSDR_INCLUDE_DIR}")

find_package(libusb-1.0 REQUIRED)

# Adding executable and linking againts the appropriate libraries (rtlsdr and pthread should suffice)
add_executable(es_tcp main.cc convenience/convenience.h convenience/convenience.c converter/converter.h converter/converter.c)
target_link_libraries(es_tcp
    pthread
    ${RTLSDR_LIBRARY}
    ${LIBUSB_LIBRARIES}
)