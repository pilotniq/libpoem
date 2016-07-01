#
# This file inspired by blog at https://rix0r.nl/blog/2015/08/13/cmake-guide/
#
# Requires bug fixes in CMake 3.5 for XCode project generation, I think.
cmake_minimum_required(VERSION 3.5)
cmake_policy( SET CMP0048 NEW )

project(libpoem VERSION 0.1 LANGUAGES C ASM)

INCLUDE_DIRECTORIES( inc )

# set(CMAKE_C_FLAGS "-Wall")
# set(CMAKE_C_FLAGS "-Wall")

set(LIB_LIST "src/error.c" src/circularBuffer.c src/uBlox6/gps_ublox6.c src/serial.c src/system.c)

if(UNIX)
  set(LIB_LIST ${LIB_LIST} "src/unix/serial_unix.c" "src/unix/error_unix.c" )
  add_executable( ubloxTest src/ubloxTest.c )
  target_link_libraries( ubloxTest poem )
  add_executable( gpsTest test/unix/gpsTest.c )
  target_link_libraries( gpsTest poem )
endif()

if( CMAKE_SYSTEM_PROCESSOR STREQUAL "nRF5" )
  set(LIB_LIST ${LIB_LIST} "src/nRF5/error_nrf.c" "src/nRF5/serial_nRF5_app.c" 
                           "src/nRF5/system_nRF5.c" "src/nRF5/logging_nRF5.c"
			   "src/nRF5/i2c_app_nRF5.c" "src/lsm303dlhc.c"
       "${SDK_COMPONENTS}/drivers_nrf/common/nrf_drv_common.c"
       "${SDK_COMPONENTS}/drivers_nrf/delay/nrf_delay.c"
       "${SDK_COMPONENTS}/drivers_nrf/twi_master/nrf_drv_twi.c"
       "${SDK_COMPONENTS}/drivers_nrf/uart/nrf_drv_uart.c" 
       "${SDK_COMPONENTS}/libraries/timer/app_timer.c"
       "${SDK_COMPONENTS}/libraries/twi/app_twi.c"
       "${SDK_COMPONENTS}/libraries/util/app_error.c"
       "${SDK_COMPONENTS}/libraries/util/app_util_platform.c"
       "${SDK_COMPONENTS}/libraries/util/nrf_log.c"
       "${SDK_COMPONENTS}/softdevice/common/softdevice_handler/softdevice_handler.c"
       "${NRF_SDK}/external/segger_rtt/SEGGER_RTT.c"
       "${NRF_SDK}/external/segger_rtt/SEGGER_RTT_printf.c"
       "${NRF_TOOLCHAIN}/system_nrf51.c"
       "${NRF_GCC}/gcc_startup_nrf51.s" )
   set_source_files_properties(${NRF_GCC}/gcc_startup_nrf51.s PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")
# for segger vprintf
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNRF_LOG_USES_RTT")

# below needed for nRF library to find nrf_...config...h file
  INCLUDE_DIRECTORIES( inc/poem/nRF5 )
#  add_executable( ubloxTest src/ubloxTest.c )
#  // target_link_libraries( ubloxTest poem )
#  // add_executable( gpsTest test/gpsTest.c )
#  // target_link_libraries( gpsTest poem )
   include_directories( "${NRF_SDK}/external/segger_rtt" )
#   add_executable( serialWrite test/nRF5/serialWrite.c ${NRF_GCC}/gcc_startup_nrf51.s)
   add_executable( serialWrite test/nRF5/serialWrite.c)
   target_link_libraries( serialWrite c nosys m poem )

   add_executable( serialRead test/nRF5/serialRead.c)
   target_link_libraries( serialRead c nosys m poem )

   add_executable( logTest test/nRF5/logTest.c)
   target_link_libraries( logTest c nosys m poem )

   add_executable( gpsTest test/nRF5/gpsTest.c)
   target_link_libraries( gpsTest c nosys m poem )

   add_executable( lsm303test test/nRF5/lsm303test.c)
   target_link_libraries( lsm303test c nosys m poem )
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


