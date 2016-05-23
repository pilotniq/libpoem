#
# This file inspired by blog at https://rix0r.nl/blog/2015/08/13/cmake-guide/
#
cmake_minimum_required(VERSION 2.8)
cmake_policy( SET CMP0048 NEW )

project(libpoem VERSION 0.1 LANGUAGES C)

INCLUDE_DIRECTORIES( inc )

set(CMAKE_CXX_FLAGS "-Wall")

set(LIB_LIST "src/error.c" src/uBlox6/gps_ublox6.c src/serial.c)

if(UNIX)
  set(LIB_LIST ${LIB_LIST} "src/unix/serial_unix.c" "src/unix/error_unix.c" )
  add_executable( ubloxTest src/ubloxTest.c )
  target_link_libraries( ubloxTest poem )
  add_executable( gpsTest test/gpsTest.c )
  target_link_libraries( gpsTest poem )
endif()

file(GLOB_RECURSE INCS "*.h")

add_library( poem ${LIB_LIST} ${INCS} )

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(poem PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries( poem )

# 'make install' to the correct location
install(TARGETS poem
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)  # This is for Windows
install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the build directory
export(TARGETS poem FILE MyLibraryConfig.cmake)

if(UNIX)
  # Every library has unit tests, of course
  add_executable(testlib
    test/testpoem.c)

  target_link_libraries(testlib poem)
  add_test(testlib testlib)
endif()

# Put build results in the directory where the CMakeLists.txt is
# set_target_properties(poem PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Add '_d' after debug builds
set_target_properties(poem PROPERTIES DEBUG_POSTFIX "_d")


