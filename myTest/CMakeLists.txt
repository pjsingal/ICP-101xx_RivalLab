cmake_minimum_required(VERSION 3.12) # Set minimum required version of CMake
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake) # Include build functions from Pico SDK

project(myTest C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init() # Creates a pico-sdk subdirectory in our project for the libraries

add_executable(${PROJECT_NAME} tester.c) # Tell CMake where to find the executable source file
pico_add_extra_outputs(${PROJECT_NAME}) # Create map/bin/hex/uf2 files
target_link_libraries(${PROJECT_NAME} pico_stdlib) # Link to pico_stdlib (gpio, time, etc. functions)