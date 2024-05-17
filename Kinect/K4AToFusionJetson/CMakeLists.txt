cmake_minimum_required(VERSION 3.10)
## use this to globally use C++17 with in our project
set(CMAKE_CXX_STANDARD 17)
## load in pkg-config support
find_package(PkgConfig)

project(AzureKinectNanoToFusion LANGUAGES CXX)

# Find the K4A Libraries
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)
message("Finding K4A SDK Binaries")
unset(k4a_DIR CACHE)

find_package(k4a 1.4.0 QUIET MODULE REQUIRED)
set(K4A_LIBS k4a::k4a;k4a::k4arecord)

# Try to find and enable the body tracking SDK
find_package(k4abt 1.0.0 QUIET MODULE)
if (k4abt_found)
    list(APPEND K4A_LIBS k4abt::k4abt)
    message(STATUS "Body Tracking SDK found: compiling support for body tracking")
    target_compile_definitions(${PROJECT_NAME}_node PUBLIC K4A_BODY_TRACKING)
    target_compile_definitions(${PROJECT_NAME}_nodelet PUBLIC K4A_BODY_TRACKING)
else()
    message("!!! Body Tracking SDK not found: body tracking features will not be available !!!")
endif()

include(Installk4a)

## Add ZMQ Dependencies
#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)
option(ENABLE_DRAFTS "Enable Draft APIs" ON)

## Add OpenCV Dependencies
find_package(OpenCV 4.5.4 REQUIRED)

## Load BOOST for networking
find_package(Boost COMPONENTS iostreams system)

## Add threading
find_package(Threads)

## Add nurses
find_package(Curses)

find_package(SoundIo)

## Add OpenMP
FIND_PACKAGE( OpenMP)
  if(OPENMP_FOUND)
     message("OPENMP FOUND")
     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()

set(PEABODY_NETWORKING_HEADERS
    ${PEABODY_NETWORKING}/ControlPanelConnector.h
    ${PEABODY_NETWORKING}/NetworkingDataTypes.h
    ${PEABODY_NETWORKING_ARM}/Win2Linux.h
    ${PEABODY_NETWORKING_ARM}/fusion_srv/TCPServer.h
    ${PEABODY_NETWORKING_ARM}/fusion_srv/DecodedCompressor.h
    ${PEABODY_NETWORKING_ARM}/fusion_srv/ICompressor.h
)
set(PEABODY_NETWORKING_SOURCE
    ${PEABODY_NETWORKING}/ControlPanelConnector.cpp
    ${PEABODY_NETWORKING_ARM}/fusion_srv/DecodedCompressor.cpp
)

set(K4A_SDK_DIR /home/peabody/Azure-Kinect-Sensor-SDK)
set(K4A_SDK_AUDIO_DIR ${K4A_SDK_DIR}/tools/k4aviewer)
set(K4A_SDK_AUDIO_PLATFORM_DIR ${K4A_SDK_DIR}/tools/k4aviewer/platform/linux)

set(K4A_AUDIO_HEADERS
    ${K4A_SDK_AUDIO_DIR}/k4aaudiomanager.h
    ${K4A_SDK_AUDIO_DIR}/k4adevicecorrelator.h
    ${K4A_SDK_AUDIO_DIR}/k4amicrophone.h
    ${K4A_SDK_AUDIO_DIR}/k4amicrophonelistener.h
    ${K4A_SDK_AUDIO_DIR}/k4asoundio_util.h
)

set(K4A_AUDIO_SOURCE
    ${K4A_SDK_AUDIO_DIR}/k4aaudiomanager.cpp
    ${K4A_SDK_AUDIO_PLATFORM_DIR}/k4adevicecorrelator.cpp
    ${K4A_SDK_AUDIO_DIR}/k4amicrophone.cpp
    ${K4A_SDK_AUDIO_DIR}/k4amicrophonelistener.cpp
)

set(HEADER_FILES 
    K4ASourceManager.h
    SourceStats.h    
    Util.h
    ${PEABODY_DIR}/McLib2/LibUtility/include/Logger.h
    K4ACaptureSource.h
    K4AControlPanelConnector.h
    ${PEABODY_NETWORKING_HEADERS}
    ${K4A_AUDIO_HEADERS}
    ${PEABODY_DIR}/McLib2/LibUtility/include/Config.h
    ${PEABODY_DIR}/McLib2/LibUtility/include/Macros.h
    )

set(SOURCES 
    K4AToFusion.cpp
    K4ASourceManager.cpp
    SourceStats.cpp
    Util.cpp
    K4ACaptureSource.cpp
    K4AControlPanelConnector.cpp
    ${PEABODY_NETWORKING_SOURCE}
    ${K4A_AUDIO_SOURCE}
    )

include_directories(     
    ${PEABODY_DIR}/McLib2/LibUtility/include # MUST INCLUDE BEFORE ${PEABODY_NETWORKING} DUE TO DEPENDENCIES TO LOGGER
    ${PEABODY_NETWORKING_ARM}
    ${PEABODY_NETWORKING_ARM}/fusion_srv
    ${PEABODY_NETWORKING}  #MUST INCLUDE AFTER ${PEABODY_NETWORKING_ARM}
    ${PEABODY_DIR}/McLib2/
    ${PEABODY_DIR}/LED\ Display/    
    ${K4A_SDK_AUDIO_DIR}
    ${K4A_SDK_AUDIO_PLATFORM_DIR}
    ${SOUNDIO_INCLUDE_DIR}
    ${OpenMP_INCLUDE_PATH}
    /usr/include/libusb-1.0
    /usr/include/jsoncpp
    /usr/include/soundio    
    )

add_executable(AzureKinectNanoToFusion ${SOURCES} ${HEADER_FILES})

target_include_directories(AzureKinectNanoToFusion PUBLIC 
    ${ZeroMQ_INCLUDE_DIR} 
    ${PEABODY_DIR}/Kinect/Common     
    )

target_link_libraries(AzureKinectNanoToFusion PUBLIC 
    pthread
    cppzmq
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${K4A_LIBS}
    ${CMAKE_BINARY_DIR}/McLib2/LibUtility/libUtility.a
    ${CMAKE_BINARY_DIR}/Kinect/DepthPodToFusionARM/libDepthPodToFusionArm.a
    ${CURSES_LIBRARIES}
    ${Boost_LIBRARIES}
    stdc++fs
    ${SOUNDIO_LIBRARY}
    jsoncpp
    ${OpenMP_CXX_LIBRARIES}
    )

target_compile_definitions(AzureKinectNanoToFusion PUBLIC ZMQ_BUILD_DRAFT_API)

install(TARGETS AzureKinectNanoToFusion
    DESTINATION /usr/local/bin)

