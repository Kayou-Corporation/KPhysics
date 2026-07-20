# --------------------------------------- #
#            KPHYSICS CMAKE               #
# --------------------------------------- #

cmake_minimum_required(VERSION 3.20)

# Define Project Name
project(KEngine VERSION 0.0.1 LANGUAGES CXX)

set(KP_TARGET_NAME KPhysics)

project(${KP_TARGET_NAME} VERSION 0.0.1 LANGUAGES CXX)

# Define output build folder
set(OUTPUT_DIR_BASE "${CMAKE_SOURCE_DIR}/Builds/${CMAKE_CXX_COMPILER_ID}_$<CONFIG>")

# Define Output Build path for libraries & executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR_BASE}/Libs")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR_BASE}/Libs")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR_BASE}")

# --------------------------------------- #
#              UTILS                      #
# --------------------------------------- #
set(KP_INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Include")
set(KP_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Source")

file(GLOB_RECURSE KP_SOURCES "${KP_SOURCE_PATH}/*.cpp")

# --------------------------------------- #
#              FIND PACKAGES              #
# --------------------------------------- #
add_subdirectory(Libs)

#---------------------------------------#
#            BUILD MODE                 #
#---------------------------------------#
add_library(${KP_TARGET_NAME} SHARED ${KP_SOURCES})

#---------------------------------------#
#                COMPILE MODE           #
#---------------------------------------#
set(KP_COMPILE_OPTIONS
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)

set(KP_COMPILE_FEATURES cxx_std_23)

target_compile_features(${KP_TARGET_NAME} PUBLIC ${KP_COMPILE_FEATURES})
target_compile_options(${KP_TARGET_NAME} PUBLIC ${KP_COMPILE_OPTIONS})
target_compile_definitions(${KP_TARGET_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
target_compile_definitions(${KP_TARGET_NAME} PRIVATE _CRT_SECURE_NO_DEPRECATE)

# Disable C and C++ compiler extensions.
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Build position independent code.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Disable C and C++ compiler extensions.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#---------------------------------------#
#            DEFAULT ENGINE             #
#---------------------------------------#
target_include_directories(${KP_TARGET_NAME} PUBLIC ${KP_INCLUDE_PATH})
target_link_libraries(${KP_TARGET_NAME} PUBLIC KUtils)

#---------------------------------------#
#        EXPORTS / IMPORTS FOR DLL      #
#---------------------------------------#
# Disable lib prefix
set_target_properties(${KP_TARGET_NAME} PROPERTIES PREFIX "")

# Set dll name
set_target_properties(${KP_TARGET_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}-${PROJECT_VERSION}")

# Set export macro
target_compile_definitions(${KP_TARGET_NAME} PRIVATE EXPORTS)

# --------------------------------------- #
#              ENABLE TESTING             #
# --------------------------------------- #
# Enable CTests
enable_testing()

option(KP_BUILD_EXECUTABLE "Build an executable to live debug the queue" ON)

if (KP_BUILD_EXECUTABLE)
    add_subdirectory(Test)
endif ()