cmake_minimum_required(VERSION 3.2)
cmake_policy( SET CMP0048 NEW )

project(poem VERSION 0.1 LANGUAGES C ASM)

# set(CMAKE_CXX_FLAGS "-Wall")

# add_library( poem error.c unix.c serial_osx.c )
# add_executable( ubloxTest ubloxTest.c )
# target_link_libraries( ubloxTest poem )

# Put build results in the directory where the CMakeLists.txt is
# set_target_properties(poem PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Add '_d' after debug builds
# set_target_properties(poem PROPERTIES DEBUG_POSTFIX "_d")

# Targets that we develop
add_subdirectory(libpoem)

