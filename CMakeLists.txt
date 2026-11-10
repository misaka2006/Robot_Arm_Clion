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
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Robot_Arm_Clion)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include_directories(
        RobotArm/Inc
)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        RobotArm/Src/rbt_Joint.cpp
        RobotArm/Inc/rbt_Joint.h
        User/Algorithm/Src/alg_matrix.cpp
        User/Algorithm/Inc/alg_matrix.h
        User/Algorithm/Src/alg_RNEA.cpp
        User/Algorithm/Inc/alg_RNEA.h)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    SUFFIX ".axf"
    LINKER_LANGUAGE CXX
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE USER_CPP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.cpp"
)
file(GLOB_RECURSE USER_MODULE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*/Src/*.cpp"
)
# 当前仅测试 Class_Joint_DM_Motor，不编译旧的整臂控制实现。
list(REMOVE_ITEM USER_MODULE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/User/Chariot/Src/crt_robot_arm.cpp"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_CPP_SOURCES}
    ${USER_MODULE_SOURCES}
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
    # Add user defined include paths
)
foreach(USER_MODULE Algorithm Chariot Congfig Device Driver Interaction Task UI)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/User/${USER_MODULE}/Inc
    )
endforeach()

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfsp_math.a
    m

    # Add user defined libraries
)


# Keep the AXF (ELF with debug symbols) for Ozone and generate Intel HEX for flashing.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    VERBATIM
)
