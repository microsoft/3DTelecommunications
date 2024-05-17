cmake_minimum_required(VERSION 3.5)
## use this to globally use C++11 with in our project
set(CMAKE_CXX_STANDARD 11)
## load in pkg-config support
find_package(PkgConfig)
set(CMAKE_FIND_LIBRARY_PREFIXES lib)
set(CMAKE_FIND_LIBRARY_SUFFIXES so)

project(LibUtility LANGUAGES CXX CUDA)

## Add OpenCV Dependencies
find_package(OpenCV 4.5.4 REQUIRED)
## Add threading
#find_package(Threads)

set(HEADER_FILES 
    include/Config.h
    include/Logger.h
    include/Macros.h
    )

set(SOURCES 
    src/Config.cpp
    src/Logger.cpp
    )

include_directories( 
	${PEABODY_DIR}/Networking_Linux_ARM
    ${OpenCV_INC}
     )

add_library(Utility ${SOURCES} ${HEADER_FILES})
target_include_directories(Utility PUBLIC     
    include/
    )
target_link_libraries(Utility ${OpenCV_LIBS})


install(TARGETS Utility DESTINATION ~/K4AToFusion)
