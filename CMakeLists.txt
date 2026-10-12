cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rocket_avioniks)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/bmp280.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/avionics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/flight.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/ek7.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/gps.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/lora.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/runmode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/mahony.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    # LoRa link (Src/lora.c) compiles its HAL transmit path only with this;
    # without it the same source builds as the HAL-free core used by
    # tests/host/test_lora_host.c.
    LORA_WITH_HAL
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Bench/SIT-SUT builds: define FLIGHT_TEST_MODE=1 to disable the 1250 m apogee
# gate, arm the bench fall triggers (BLUE LED) and override the main altitude.
# Production launches leave this OFF so the flight gates stay untouched.
#   cmake -B build/Bench -DCMAKE_BUILD_TYPE=Debug -DFLIGHT_TEST_MODE=1
option(FLIGHT_TEST_MODE "Bench mode: 0 m apogee gate + armed fall triggers" OFF)
if(FLIGHT_TEST_MODE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FLIGHT_TEST_MODE=1)
    message(STATUS "Bench mode ON (FLIGHT_TEST_MODE=1)")
endif()

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
