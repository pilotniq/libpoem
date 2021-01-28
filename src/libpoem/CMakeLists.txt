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
set(CMAKE_C_FLAGS "-Wno-unused-label" )
set(LIB_LIST "src/error.c" src/circularBuffer.c src/uBlox6/gps_ublox6.c src/serial.c src/system.c)

if(UNIX)
  set(LIB_LIST ${LIB_LIST} "src/unix/serial_unix.c" "src/unix/error_unix.c" )
  add_executable( ubloxTest src/ubloxTest.c )
  target_link_libraries( ubloxTest poem )
  add_executable( gpsTest test/unix/gpsTest.c )
  target_link_libraries( gpsTest poem )
endif()

# drivers_nrf/...: nrf_drv_common.c nrf_delay.c
# SOFTDEVICE_HANDLER_INIT "${SDK_COMPONENTS}/libraries/twi/app_twi.c"
# "${SDK_COMPONENTS}/libraries/util/nrf_log.c"
# "${SDK_COMPONENTS}/softdevice/common/softdevice_handler/softdevice_handler.c"
# "src/nRF5/i2c_app_nRF5.c" 
# "src/nRF5/serial_nRF5_app.c" 
if( CMAKE_SYSTEM_PROCESSOR STREQUAL "nRF5" )
  set(LIB_LIST ${LIB_LIST} "src/nRF5/error_nrf.c" 
                           "src/nRF5/system_nRF5.c" 
                           "src/nRF5/logging_nRF5.c"
                           "src/nRF5/spi_nrf5.c"
                           "src/nRF5/gpio_nrf5.c"
                           "src/nRF5/i2c_master_twiManager.c"
                           "src/lsm303dlhc.c"
                           "src/si705x.c"
                           "src/waveshare/eInk_waveshare.c"
       "${NRF_SDK}/integration/nrfx/legacy/nrf_drv_spi.c"
       "${NRF_SDK}/integration/nrfx/legacy/nrf_drv_twi.c"
       "${NRF_SDK}/integration/nrfx/legacy/nrf_drv_uart.c"
       "${SDK_COMPONENTS}/ble/common/ble_srv_common.c"
       "${SDK_COMPONENTS}/ble/common/ble_conn_state.c"
       "${SDK_COMPONENTS}/libraries/atomic/nrf_atomic.c"
       "${SDK_COMPONENTS}/libraries/atomic_flags/nrf_atflags.c"
       "${SDK_COMPONENTS}/libraries/balloc/nrf_balloc.c"
       "${SDK_COMPONENTS}/libraries/experimental_section_vars/nrf_section_iter.c"
       "${SDK_COMPONENTS}/libraries/log/src/nrf_log_frontend.c"
       "${SDK_COMPONENTS}/libraries/memobj/nrf_memobj.c"
       "${SDK_COMPONENTS}/libraries/pwr_mgmt/nrf_pwr_mgmt.c"
       "${SDK_COMPONENTS}/libraries/queue/nrf_queue.c"
       "${SDK_COMPONENTS}/libraries/ringbuf/nrf_ringbuf.c"
       "${SDK_COMPONENTS}/libraries/strerror/nrf_strerror.c"
       "${SDK_COMPONENTS}/libraries/timer/app_timer.c"
       "${SDK_COMPONENTS}/libraries/twi_mngr/nrf_twi_mngr.c"
       "${SDK_COMPONENTS}/libraries/util/app_error.c"
       "${SDK_COMPONENTS}/libraries/util/app_error_handler_gcc.c"
       "${SDK_COMPONENTS}/libraries/util/app_error_weak.c"
       "${SDK_COMPONENTS}/libraries/util/app_util_platform.c"
       "${SDK_COMPONENTS}/softdevice/common/nrf_sdh.c"
       "${SDK_COMPONENTS}/softdevice/common/nrf_sdh_ble.c"
       "${NRF_SDK}/modules/nrfx/drivers/src/nrfx_spi.c"
       "${NRF_SDK}/modules/nrfx/drivers/src/nrfx_twi.c"
       "${NRF_SDK}/modules/nrfx/drivers/src/nrfx_twi_twim.c"
       "${NRF_SDK}/modules/nrfx/drivers/src/nrfx_twim.c"
       "${NRF_SDK}/modules/nrfx/drivers/src/prs/nrfx_prs.c"
       "${NRF_SDK}/modules/nrfx/mdk/gcc_startup_nrf52840.S"
       "${NRF_SDK}/modules/nrfx/mdk/system_nrf52840.c"
       "${NRF_SDK}/external/fprintf/nrf_fprintf.c"
       "${NRF_SDK}/external/fprintf/nrf_fprintf_format.c"
       "${NRF_SDK}/external/segger_rtt/SEGGER_RTT.c"
       "${NRF_SDK}/external/segger_rtt/SEGGER_RTT_printf.c"
       )

#       "${NRF_TOOLCHAIN}/system_nrf51.c"
#       "${NRF_GCC}/gcc_startup_nrf51.s" )
#   set_source_files_properties(${NRF_GCC}/gcc_startup_nrf51.s PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")
# for segger vprintf
  # Note: below sets 52840 as chip
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNRF_LOG_USES_RTT -DNRF_SDK_VERSION_MAJOR=16 -DNRF52840_XXAA -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")

# below needed for nRF library to find nrf_...config...h file
  INCLUDE_DIRECTORIES( inc/poem/nRF5 )
#  INCLUDE_DIRECTORIES( "${NRF_SDK}/components/softdevice/${SOFTDEVICE}" )
  
#  add_executable( ubloxTest src/ubloxTest.c )
#  // target_link_libraries( ubloxTest poem )
#  // add_executable( gpsTest test/gpsTest.c )
#  // target_link_libraries( gpsTest poem )
   include_directories( "${NRF_SDK}/external/segger_rtt" )
   include_directories( "${NRF_SDK}/components/libraries/atomic" )
   include_directories( "${NRF_SDK}/components/libraries/atomic_flags" )
   include_directories( "${NRF_SDK}/components/libraries/balloc" )
   include_directories( "${NRF_SDK}/components/libraries/delay" )
   include_directories( "${NRF_SDK}/components/libraries/experimental_section_vars" )
   include_directories( "${NRF_SDK}/components/libraries/queue" )
   include_directories( "${NRF_SDK}/components/libraries/log" )
   include_directories( "${NRF_SDK}/components/libraries/log/src" )
   include_directories( "${NRF_SDK}/components/libraries/memobj" )
   include_directories( "${NRF_SDK}/components/libraries/mutex" )
   include_directories( "${NRF_SDK}/components/libraries/ringbuf" )
   include_directories( "${NRF_SDK}/components/libraries/strerror" )
   include_directories( "${NRF_SDK}/components/libraries/queue" )
   include_directories( "${NRF_SDK}/components/libraries/timer" )
   include_directories( "${NRF_SDK}/components/libraries/twi_mngr" )
   include_directories( "${NRF_SDK}/components/libraries/util" )
   include_directories( "${NRF_SDK}/components/softdevice/common" )
   include_directories("${NRF_SDK}/components/toolchain/cmsis/include")
   include_directories("${NRF_SDK}/external/fprintf")
   include_directories("${NRF_SDK}/integration/nrfx/legacy")
   include_directories("${NRF_SDK}/integration/nrfx")
   include_directories("${NRF_SDK}/modules/nrfx")
   include_directories("${NRF_SDK}/modules/nrfx/mdk")
   include_directories("${NRF_SDK}/modules/nrfx/drivers/include")
   include_directories("${NRF_SDK}/modules/nrfx/drivers/src/prs")
   include_directories("${NRF_SDK}/modules/nrfx/hal")

   # for nrf_error.h:
   include_directories("${NRF_SDK}/components/softdevice/${SOFTDEVICE}/headers")
   include_directories("${NRF_SDK}/components/softdevice/${SOFTDEVICE}/headers/nrf52")

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
SET_TARGET_PROPERTIES(poem PROPERTIES LINKER_LANGUAGE C)

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


