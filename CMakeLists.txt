cmake_minimum_required(VERSION 3.5)
## use this to globally use C++11 with in our project
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CUDA_CUDART_LIBRARY /usr/lcoal/cuda/lib64)

add_compile_options(-Wall -Werror -fopenmp)
project(Peabody LANGUAGES CXX)

## load in pkg-config support
find_package(PkgConfig)

set(PEABODY_DIR ${CMAKE_CURRENT_SOURCE_DIR})

include(GetGitRevisionDescription)
git_sha1(VERSION_SHA1)
git_version(VERSION)

execute_process( COMMAND git log -1 --format=%ct OUTPUT_VARIABLE COMMIT_TS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process( COMMAND powershell -File ${PEABODY_DIR}/Scripts/CreateIncludeFileFromGitVersion.ps1 ${PEABODY_DIR})
 
## Assumes you're using a build/release folder structure, or building with azure pipeline which uses this structure
set(PEABODY_DEPENDENCIES ${PEABODY_DIR}/../Dependencies)
set(PEABODY_NETWORKING_ARM ${PEABODY_DIR}/Networking_Linux_ARM)
set(PEABODY_NETWORKING ${PEABODY_DIR}/Networking)

set(PEABODY_NETWORKING_HEADERS
    ${PEABODY_NETWORKING_ARM}/Win2Linux.h
    ${PEABODY_NETWORKING}/fusion_srv/ConnectionManager.h
    ${PEABODY_NETWORKING_ARM}/fusion_srv/DecodedCompressor.h
    ${PEABODY_NETWORKING_ARM}/fusion_srv/ICompressor.h
)
set(PEABODY_NETWORKING_SOURCE
    ${PEABODY_NETWORKING}/fusion_srv/ConnectionManager.cpp
    ${PEABODY_NETWORKING_ARM}/fusion_srv/DecodedCompressor.cpp
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    message("****Setting build type to 'Release' as none was specified.")
else()
    message("****Build type is ${CMAKE_BUILD_TYPE}")
endif()

## This is where the version.h file will be in the linux build (see the powershell line above)
include_directories( ${PEABODY_DIR} )
	
add_subdirectory(McLib2/LibUtility)
add_subdirectory(Kinect/DepthPodToFusionARM)


set(PEABODY_INT_DIR ${PEABODY_DIR}/build/release)

link_directories(
    ${PEABODY_INT_DIR}/McLib2/LibUtility
    ${PEABODY_INT_DIR}/Kinect/DepthPodToFusionARM
)

add_subdirectory(Kinect/K4AToFusionJetson)
add_subdirectory(Kinect/K4ALauncherDaemon)
add_subdirectory(Kinect/K4ARecorder)

set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Holoportation team at Microsoft Research Special Projects")
set(CPACK_PACKAGE_NAME "AzureKinectNanoToFusion")
set(CPACK_DEBIAN_PACKAGE_VERSION ${VERSION_SHORT})
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libk4a1.4 (>= 1.4), k4a-tools (>= 1.4)")
set(CPACK_DEBIAN_PACKAGE_PROVIDES "AzureKinectNanoToFusion, AKLauncherDaemon, K4ARecorder")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "AzureKinect running on NVidia Jetson Nano ingest distributor to Microsoft Research's Holoportation Fusion 4D System.")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "/usr/local/bin")
set(CPACK_PACKAGE_VENDOR "Microsoft Research Special Projects")
set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})


INCLUDE(CPack)


