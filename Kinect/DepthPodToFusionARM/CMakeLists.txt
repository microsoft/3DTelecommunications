cmake_minimum_required(VERSION 3.5)
## use this to globally use C++11 with in our project
set(CMAKE_CXX_STANDARD 17)
## load in pkg-config support
find_package(PkgConfig)

project(DepthPodToFusionArm LANGUAGES CXX)

## Load BOOST for networking
find_package(Boost COMPONENTS iostreams system)
## Add threading
find_package(Threads)
## Compressor rquires LZ4
##find_package(LibLZ4)

set(PEABODY_NETWORKING_HEADERS
    ${PEABODY_NETWORKING_ARM}/Win2Linux.h
    ${PEABODY_NETWORKING_ARM}/fusion_srv/TCPServer.h
    ${PEABODY_NETWORKING_ARM}/fusion_srv/ICompressor.h
)
set(PEABODY_NETWORKING_SOURCE
    ${PEABODY_NETWORKING_ARM}/fusion_srv/TCPServer.cpp
    ${PEABODY_NETWORKING_ARM}/fusion_srv/DecodedCompressor.cpp
)

set(HEADER_FILES 
    DepthPodToFusionAPI.h
    ${PEABODY_NETWORKING_HEADERS}
    )

set(SOURCES 
    DepthPodToFusion.cpp
    ${PEABODY_NETWORKING_SOURCE}
    )

add_library(DepthPodToFusionArm ${SOURCES} ${HEADER_FILES})
target_include_directories(DepthPodToFusionArm PUBLIC 
    ${PEABODY_DIR}/Kinect/Common 
    ${PEABODY_DIR}/Networking_Linux_ARM
    ${PEABODY_DIR}/Networking_Linux_ARM/fusion_srv
    ${LIBLZ4_INCLUDE_DIRS}
    )

target_link_libraries(DepthPodToFusionArm PUBLIC 
    pThread
    #${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${LIBLZ4_LIBRARIES}
    )

install(TARGETS DepthPodToFusionArm DESTINATION ~/K4AToFusion)

