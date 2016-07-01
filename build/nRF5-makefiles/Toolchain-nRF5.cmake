INCLUDE(CMakeForceCompiler)

# this one is important
# Not sure if it should be Generic or nRF5 so we can select files depending on it later.
SET(CMAKE_SYSTEM_NAME Generic)
#this one not so much
SET(CMAKE_SYSTEM_VERSION 1)
SET(CMAKE_SYSTEM_PROCESSOR nRF5)

# specify the cross compiler
# The C compiler can't link, complains it is missing _exit, so we need to force the compiler
# Apparently there is a fix coming in cmake 3.6, see https://cmake.org/pipermail/cmake-developers/2016-February/027889.html
CMAKE_FORCE_C_COMPILER( /usr/local/gcc-arm-none-eabi-5_2-2015q4/bin/arm-none-eabi-gcc GNU)
#set( CMAKE_C_COMPILER /usr/local/gcc-arm-none-eabi-5_2-2015q4/bin/arm-none-eabi-gcc )
#SET(CMAKE_CXX_COMPILER /opt/eldk-2007-01-19/usr/bin/ppc_74xx-g++)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH  /usr/local/gcc-arm-none-eabi-5_2-2015q4/)

# specifiy target cpu flags

#set(PLATFORM_CONFIG_C_FLAGS    "--specs=nosys.specs" CACHE STRING "platform config c flags")
#SET(CMAKE_C_FLAGS "${PLATFORM_CONFIG_C_FLAGS} ${CMAKE_C_FLAGS}")
#SET(CMAKE_C_FLAGS "--specs=nosys.specs")
#SET(CMAKE_EXE_LINKER_FLAGS "--specs=nosys.specs")

# Should be parameters
set( CHIP        NRF51 )
set( SOFTDEVICE  S130 )

set( NRF_SDK        $ENV{HOME}/src/nRF51822/SDK11 )
set( SDK_COMPONENTS ${NRF_SDK}/components )
set( NRF_TOOLCHAIN  ${SDK_COMPONENTS}/toolchain )
set( NRF_GCC        ${NRF_TOOLCHAIN}/gcc )

set(LINKER_SCRIPT ${SDK_COMPONENTS}/softdevice/s130/toolchain/armgcc/armgcc_s130_nrf51822_xxaa.ld)

# It's best to hide all the details of setting up the variable SRCS in a CMake
# macro. The macro can then be called in all the project CMake list files to add
# sources.
#
# The macro first computes the path of the source file relative to the project
# root for each argument. If the macro is invoked from inside a project sub
# directory the new value of the variable SRCS needs to be propagated to the
# parent folder by using the PARENT_SCOPE option.
#
# Source: http://stackoverflow.com/questions/7046956/populating-srcs-from-cmakelists-txt-in-subdirectories
macro (add_sources)
    file (RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach (_src ${ARGN})
        if (_relPath)
            list (APPEND SRCS "${_relPath}/${_src}")
        else()
            list (APPEND SRCS "${_src}")
        endif()
    endforeach()
    if (_relPath)
        # propagate to parent directory
        set (SRCS ${SRCS} PARENT_SCOPE)
    endif()
endmacro()

# CFlags don't seem to make it into the XCode project. Trying the CACHE STRING thing to see if it will work.
# see https://cmake.org/pipermail/cmake/2010-December/041378.html
# SET(CMAKE_C_FLAGS "--specs=nosys.specs")
set(CMAKE_C_FLAGS "-T${LINKER_SCRIPT} -DDEBUG -DBOARD_CUSTOM -DSOFTDEVICE_PRESENT -D${CHIP} -D${SOFTDEVICE} -DBLE_STACK_SUPPORT_REQD -DBSP_DEFINES_ONLY -mcpu=cortex-m0 -mthumb -mabi=aapcs --std=gnu99 -Wall -Werror -O3 -mfloat-abi=soft -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin --short-enums -g -O0 -L ${SDK_COMPONENTS}/toolchain/gcc -lc -lnosys" CACHE STRING "" FORCE)
set(ASM_OPTIONS "-x assembler-with-cpp")
set(CMAKE_ASM_FLAGS "${CFLAGS} ${ASM_OPTIONS}")
#set(CMAKE_ASM_FLAGS "-x assembler-with-cpp -DDEBUG -DBOARD_CUSTOM -DSOFTDEVICE_PRESENT -D${CHIP} -D${SOFTDEVICE} -DBLE_STACK_SUPPORT_REQD -DBSP_DEFINES_ONLY" )
#set(CMAKE_EXE_LINKER_FLAGS "-Xlinker -mthumb -mabi=aapcs -L /Users/erlandlewin/src/nRF51822/SDK11/components/toolchain/gcc -Tarmgcc_s130_nrf51822_xxaa.ld -mcpu=cortex-m0 -Wl,--gc-sections --specs=nano.specs" )
UNSET(CMAKE_EXE_LINKER_FLAGS CACHE)
set(CMAKE_EXE_LINKER_FLAGS "-Xlinker -mabi=aapcs -L /Users/erlandlewin/src/nRF51822/SDK11/components/toolchain/gcc -Tarmgcc_s130_nrf51822_xxaa.ld -mcpu=cortex-m0 -Wl,--gc-sections --specs=nano.specs" )
set(CMAKE_C_LINK_FLAGS "-Xlinker -Map=nrf51822_xxaa_s130.map -mthumb -mabi=aapcs -L /Users/erlandlewin/src/nRF51822/SDK11/components/toolchain/gcc -T${LINKER_SCRIPT} -mcpu=cortex-m0 -Wl,--gc-sections --specs=nano.specs" )
set(LINK_FLAGS "-Xlinker")
# This is the linker that's used.
set( CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS>  -o <TARGET> <LINK_LIBRARIES>")
set( CMAKE_EXE_LINK_EXECUTABLE "<eCMAKE_C_COMPILER> <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS>  -o <TARGET> <LINK_LIBRARIES>")
#set( CMAKE_ASM_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS>  -o <TARGET> <LINK_LIBRARIES>")
#set(CMAKE_LINKER "/usr/local/gcc-arm-none-eabi-5_2-2015q4/bin/arm-none-eabi-gcc" )
#set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_LINKER> <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS>  -o <TARGET> <LINK_LIBRARIES>")
#
# This doesn't seem to work.
#
set(SOURCE_FILES
    ${NRF_GCC}/gcc_startup_nrf51.s
    ${NRF_TOOLCHAIN}/system_nrf51.c
    ${SDK_COMPONENTS}/drivers_nrf/hal/nrf_delay.c
    ${SDK_COMPONENTS}/softdevice/common/softdevice_handler/softdevice_handler.c
    )

string( TOLOWER "${SOFTDEVICE}" SOFTDEVICE_LOWER )

INCLUDE_DIRECTORIES( ${NRF_TOOLCHAIN}/CMSIS/Include 
                     ${NRF_TOOLCHAIN} 
                     ${NRF_GCC} 
		     ${NRF_SDK}/examples/bsp
		     ${SDK_COMPONENTS}/device
		     ${SDK_COMPONENTS}/drivers_nrf/common
		     ${SDK_COMPONENTS}/drivers_nrf/delay
		     ${SDK_COMPONENTS}/drivers_nrf/hal
		     ${SDK_COMPONENTS}/drivers_nrf/twi_master
		     ${SDK_COMPONENTS}/drivers_nrf/uart
                     ${SDK_COMPONENTS}/libraries/timer
                     ${SDK_COMPONENTS}/libraries/twi
                     ${SDK_COMPONENTS}/libraries/util
                     ${SDK_COMPONENTS}/softdevice/${SOFTDEVICE_LOWER}/headers
                     ${SDK_COMPONENTS}/softdevice/common/softdevice_handler )

# below was placed in poem/inc/nRF5, adapted to application
# ${SDK_COMPONENTS}/drivers_nrf/config

add_sources(
    ${NRF_GCC}/gcc_startup_nrf51.s
    ${NRF_TOOLCHAIN}/system_nrf51.c
)


# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
