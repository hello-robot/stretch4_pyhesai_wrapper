# CMakeLists.txt

cmake_minimum_required(VERSION 3.15)
project(pyhesai_wrapper_cpp)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Check for the required Hesai libraries and tell the user to apt install them
find_library(PCAP_LIB pcap)
find_library(SSL_LIB ssl)

if(NOT PCAP_LIB OR NOT SSL_LIB)
    message(FATAL_ERROR "Missing required dependencies!\n"
                        "Please run the following command to install them:\n"
                        "sudo apt install libpcap-dev libssl-dev")
endif()

# --- Find Python ---
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# --- Fetch pybind11 automatically ---
include(FetchContent)

# Try to find pybind11 from pip installation first
if(NOT PYTHON_EXECUTABLE)
    set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
endif()

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -m pybind11 --cmakedir
  OUTPUT_VARIABLE pybind11_CMAKE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE pybind11_RESULT
  ERROR_QUIET
)

if(pybind11_RESULT EQUAL 0)
    # Found pybind11 via pip, use it
    message(STATUS "Found pybind11 via pip at: ${pybind11_CMAKE_DIR}")
    list(APPEND CMAKE_PREFIX_PATH ${pybind11_CMAKE_DIR})
    find_package(pybind11 REQUIRED)
else()
    # Fetch pybind11 from GitHub
    message(STATUS "pybind11 not found via pip, fetching from GitHub...")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.13.6
    )
    FetchContent_MakeAvailable(pybind11)
endif()

# --- Conditional Hesai SDK Clone based on Git URL ---

set(HESAI_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/HesaiLidar_SDK_2.0)
set(EXPECTED_SDK_URL "https://github.com/hello-robot/HesaiLidar_SDK_2.0.git")

# 1. If the directory exists, check if its remote URL matches what we expect
if(EXISTS ${HESAI_SDK_DIR})
    execute_process(
        COMMAND git config --get remote.origin.url
        WORKING_DIRECTORY ${HESAI_SDK_DIR}
        OUTPUT_VARIABLE CURRENT_SDK_URL
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    # If the URL is different (or git command failed), wipe the directory
    if(NOT "${CURRENT_SDK_URL}" STREQUAL "${EXPECTED_SDK_URL}")
        message(STATUS "Hesai SDK repository URL has changed or is stale. Cleaning up old directory...")
        file(REMOVE_RECURSE ${HESAI_SDK_DIR})
    endif()
endif()

# 2. Clone the repository if it doesn't exist (either it never did, or it was just wiped above)
if(NOT EXISTS ${HESAI_SDK_DIR})
    message(STATUS "Cloning HesaiLidar_SDK_2.0 from ${EXPECTED_SDK_URL}...")
    execute_process(
        COMMAND git clone --recursive ${EXPECTED_SDK_URL}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE GIT_CLONE_RESULT
    )

    if(NOT GIT_CLONE_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to clone HesaiLidar_SDK_2.0")
    endif()

    message(STATUS "HesaiLidar_SDK_2.0 cloned successfully")
endif()

# Build the Hesai SDK if not already built
set(HESAI_SDK_BUILD_DIR ${HESAI_SDK_DIR}/build)
# hesai_sdk_lib, lidar_lib, and udpParser_lib are INTERFACE (header-only) in this SDK branch
set(HESAI_SDK_LIBS
    ${HESAI_SDK_BUILD_DIR}/libhesai/PtcClient/libptcClient_lib.a
    ${HESAI_SDK_BUILD_DIR}/libhesai/Source/libsource_lib.a
    ${HESAI_SDK_BUILD_DIR}/libhesai/Common/libplatutils_lib.a
    ${HESAI_SDK_BUILD_DIR}/libhesai/Logger/liblog_lib.a
    ${HESAI_SDK_BUILD_DIR}/libhesai/SerialClient/libserialClient_lib.a
)

message(STATUS "Building Hesai SDK if needed...")

# Create build directory
file(MAKE_DIRECTORY ${HESAI_SDK_BUILD_DIR})

# Configure the SDK
execute_process(
    COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${HESAI_SDK_DIR}
    WORKING_DIRECTORY ${HESAI_SDK_BUILD_DIR}
    RESULT_VARIABLE SDK_CONFIG_RESULT
)

if(NOT SDK_CONFIG_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to configure Hesai SDK")
endif()

# Build the SDK. CMake's dependency tracking keeps this a no-op when sources
# and outputs are already current.
include(ProcessorCount)
ProcessorCount(N)
if(N EQUAL 0)
    set(N 1)
endif()

execute_process(
    COMMAND ${CMAKE_COMMAND} --build . -j ${N}
    WORKING_DIRECTORY ${HESAI_SDK_BUILD_DIR}
    RESULT_VARIABLE SDK_BUILD_RESULT
)

if(NOT SDK_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to build Hesai SDK")
endif()

message(STATUS "Hesai SDK build complete")

# --- Configure Include Directories ---

# Add the pybind11 include directory
include_directories(${pybind11_INCLUDE_DIRS})

# Add SDK root and driver
include_directories(${HESAI_SDK_DIR})
include_directories(${HESAI_SDK_BUILD_DIR}/generated)

# Add all the necessary include directories from the Hesai SDK
include_directories(
    ${HESAI_SDK_DIR}/driver
    ${HESAI_SDK_DIR}/libhesai
    ${HESAI_SDK_DIR}/libhesai/include
    ${HESAI_SDK_DIR}/libhesai/Common/include
    ${HESAI_SDK_DIR}/libhesai/Common/src
    ${HESAI_SDK_DIR}/libhesai/Container/include
    ${HESAI_SDK_DIR}/libhesai/Container/src
    ${HESAI_SDK_DIR}/libhesai/Lidar
    ${HESAI_SDK_DIR}/libhesai/lidar
    ${HESAI_SDK_DIR}/libhesai/Logger/include
    ${HESAI_SDK_DIR}/libhesai/Logger/src
    ${HESAI_SDK_DIR}/libhesai/PtcClient/include
    ${HESAI_SDK_DIR}/libhesai/PtcClient/src
    ${HESAI_SDK_DIR}/libhesai/PtcParser
    ${HESAI_SDK_DIR}/libhesai/PtcParser/include
    ${HESAI_SDK_DIR}/libhesai/PtcParser/src
    ${HESAI_SDK_DIR}/libhesai/SerialClient/include
    ${HESAI_SDK_DIR}/libhesai/SerialClient/src
    ${HESAI_SDK_DIR}/libhesai/Source/include
    ${HESAI_SDK_DIR}/libhesai/Source/src
    ${HESAI_SDK_DIR}/libhesai/UdpParser/include
    ${HESAI_SDK_DIR}/libhesai/UdpParser/src
    ${HESAI_SDK_DIR}/libhesai/UdpProtocol
    ${HESAI_SDK_DIR}/libhesai/UdpParser
)

# --- Define the Python Module ---

# This creates the target 'pyhesai_wrapper_cpp' from our wrapper source file
pybind11_add_module(pyhesai_wrapper_cpp SHARED pybind_hesai_sdk.cpp)
install(TARGETS pyhesai_wrapper_cpp
        LIBRARY DESTINATION .)

# --- Link Libraries ---

# Link our 'pyhesai_wrapper_cpp' module against the Hesai SDK libraries
target_link_libraries(pyhesai_wrapper_cpp PRIVATE
    # Hesai SDK static libraries
    ${HESAI_SDK_LIBS}

    # System dependencies required by the Hesai SDK
    pthread
    pcap
    ssl
    crypto
)
