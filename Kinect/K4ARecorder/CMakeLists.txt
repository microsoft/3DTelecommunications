cmake_minimum_required(VERSION 3.5)
## use this to globally use C++11 with in our project
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS_DEUBG "-g -O0")
## load in pkg-config support
find_package(PkgConfig)

project(K4ARecorder LANGUAGES CXX)

## Add ZMQ Dependencies
#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)
option(ENABLE_DRAFTS "Enable Draft APIs" ON)

find_path(Proc_INCLUDE_DIR
    NAMES readproc.h
    PATHS /usr/include/proc
    )

set(PEABODY_NETWORKING_HEADERS
    ${PEABODY_NETWORKING_ARM}/Win2Linux.h
    ${PEABODY_NETWORKING}/ControlPanelConnector.h
)
set(PEABODY_NETWORKING_SOURCE
    ${PEABODY_NETWORKING}/ControlPanelConnector.cpp
)

set(HEADER_FILES 
    ${PEABODY_DIR}/McLib2/LibUtility/include/Logger.h
    ${PEABODY_NETWORKING_HEADERS}
    recorder.h
    K4ARecorder.h
    ${PEABODY_DIR}/McLib2/LibUtility/include/Config.h
    )

set(SOURCES 
    ${PEABODY_NETWORKING_SOURCE}
    recorder.cpp
    K4ARecorder.cpp
    )

add_executable(K4ARecorder ${SOURCES} ${HEADER_FILES})

target_include_directories(K4ARecorder PUBLIC 
    ${PEABODY_DIR}/McLib2/LibUtility/include/
    ${PEABODY_NETWORKING_ARM}
    ${PEABODY_NETWORKING_ARM}/fusion_srv
    ${PEABODY_NETWORKING}  ##Make sure this is included AFTER the ARM folder
    ${Proc_INCLUDE_DIR}
    ${ZeroMQ_INCLUDE_DIR}
    )

target_compile_definitions(K4ARecorder PUBLIC ZMQ_BUILD_DRAFT_API)

target_link_libraries(K4ARecorder PUBLIC 
    pthread
    procps    
    stdc++fs
    cppzmq 
    k4a
    k4arecord 
    ${CMAKE_BINARY_DIR}/McLib2/LibUtility/libUtility.a
    )

install(TARGETS K4ARecorder DESTINATION /usr/local/bin)

