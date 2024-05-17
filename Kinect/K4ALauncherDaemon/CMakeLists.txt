cmake_minimum_required(VERSION 3.5)
## use this to globally use C++11 with in our project
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS_DEUBG "-g -O0")
## load in pkg-config support
find_package(PkgConfig)

project(AKLauncherDaemon LANGUAGES CXX)

## Add ZMQ Dependencies
#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)
option(ENABLE_DRAFTS "Enable Draft APIs" ON)
# Find the K4A Libraries
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)
message("Finding K4A SDK Binaries")
unset(k4a_DIR CACHE)
find_package(k4a 1.4.0 QUIET MODULE REQUIRED)
set(K4A_LIBS k4a::k4a;k4a::k4arecord)

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
    K4ALauncherDaemon.h    
    ${PEABODY_DIR}/McLib2/LibUtility/include/Config.h
    )

set(SOURCES 
    ${PEABODY_NETWORKING_SOURCE}
    K4ALauncherDaemon.cpp
    )

add_executable(AKLauncherDaemon ${SOURCES} ${HEADER_FILES})

target_include_directories(AKLauncherDaemon PUBLIC 
    ${PEABODY_DIR}/McLib2/LibUtility/include/
    ${PEABODY_NETWORKING_ARM}
    ${PEABODY_NETWORKING_ARM}/fusion_srv
    ${PEABODY_NETWORKING}  ##Make sure this is included AFTER the ARM folder
    ${Proc_INCLUDE_DIR}
    ${ZeroMQ_INCLUDE_DIR}
    /usr/include/jsoncpp
    ${PEABODY_DIR}/LED\ Display/    
    )

target_compile_definitions(AKLauncherDaemon PUBLIC ZMQ_BUILD_DRAFT_API)

target_link_libraries(AKLauncherDaemon PUBLIC 
    pthread
    procps
    stdc++fs
    cppzmq 
    jsoncpp
    ${K4A_LIBS}    
    ${CMAKE_BINARY_DIR}/McLib2/LibUtility/libUtility.a
    )

install(TARGETS AKLauncherDaemon DESTINATION /usr/local/bin)

