cmake_minimum_required(VERSION 3.1)
if(EXISTS /usr/bin/clang AND EXISTS /usr/bin/clang++)
   set(CMAKE_C_COMPILER /usr/bin/clang)
   set(CMAKE_CXX_COMPILER /usr/bin/clang++)
endif()
project(Aimer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQIRED ON)
set(PROJECT_ROOT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(TEST_PATH ${CMAKE_CURRENT_LIST_DIR}/test)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_ROOT_DIRECTORY}/bin)

message(STATUS "##########${CMAKE_SYSTEM_NAME}#########")
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(OpenCV_DIR /usr/local/opt/opencv@3/share/OpenCV)
    include_directories("/usr/local/include")
    link_directories(/usr/local/opt/opencv@3/lib)
    link_directories(/usr/local/lib)
    link_libraries(${OpenCV_LIBS})
endif()

exec_program(
    mkdir ARGS
    -p ${PROJECT_ROOT_DIRECTORY}/bin
    OUTPUT_VARIABLE MKDIR_OUT
    RETURN_VALUE MKDIR_RET)

add_definitions(-DROOT_PATH="${CMAKE_CURRENT_LIST_DIR}")
add_definitions(-D${CMAKE_SYSTEM_NAME})

# Compile Depend modules
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/libBase)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/libControl)

include_directories(${BASE_INCLUDE})
include_directories(${CONTROL_INCLUDE})

# Set Variable
set(CONTROL_LIB AngleSolver Aimer)
set(WMJ_LIBRARIES Base ${VISION_LIB} ${HARDWARE_LIB} ${CONTROL_LIB})

# Test
macro(add_test_file TEST_FILE)
    add_executable(${TEST_FILE} ${TEST_PATH}/${TEST_FILE}.cpp)
    target_link_libraries(${TEST_FILE} ${WMJ_LIBRARIES})
endmacro()

add_test_file(AimerDataTest)
# add_test_file(AimerTest)
